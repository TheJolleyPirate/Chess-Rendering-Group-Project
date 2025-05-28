#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <set>
#include <stack>
#include <map>
#include <unordered_map>
#include <bits/stdc++.h>
#include <Eigen/Dense>

#include <meshChecker.hpp>
#include <object.hpp>
#include <loadModel.hpp>

using namespace std;
using namespace Eigen;

/*by Daniel Jolley-Rogers u7511912
checks whether a given object is closed*/
bool objectClosed(const Object &object){
    //if every halfEdge has a face, and a twin then the object is closed
    for(const shared_ptr<HalfEdge> &halfEdge : object.halfEdges){
        if(!halfEdge->face){
            return false;
        }
        shared_ptr<HalfEdge> twin = halfEdge->twin.lock();
        if(!twin){
            return false;
        }
        shared_ptr<HalfEdge> twinTwin = twin->twin.lock();
        if(!twinTwin || twinTwin->id != halfEdge->id){
            return false;
        }
    }
    return true;
}

/*by Daniel Jolley-Rogers u7511912
attempts to make an object closed*/
void makeObjectClosed(Object &object){
    //tries to make object closed by deleting halfEdges which are not part of a face
    auto toDelete = remove_if(
        object.halfEdges.begin(), object.halfEdges.end(), [](shared_ptr<HalfEdge> halfEdge) {return !halfEdge->face;});
    object.halfEdges.erase(toDelete, object.halfEdges.end());
}

/*by Daniel Jolley-Rogers u7511912
checks whether a given object is fully connected*/
bool objectConnected(const Object &object){
    set<int> visited;
    shared_ptr<Vertex> startingVert = object.vertices[0];
    if(!startingVert){
        return false;
    }

    stack<shared_ptr<Vertex>> vertexStack;
    vertexStack.push(startingVert);

    while (!vertexStack.empty()) {
        shared_ptr<Vertex> vertex = vertexStack.top(); 
        vertexStack.pop();
        if (!vertex || visited.count(vertex->id) > 0){
            continue;
        }
        visited.insert(vertex->id);
        for (const shared_ptr<Vertex> &neighbour : vertex->getNeighbourVertices()) {
            vertexStack.push(neighbour);
        }
    }

    int numVisited = visited.size();
    int numVertices = object.vertices.size();
    if(numVisited == numVertices){
        return true;
    }
    return false;
}

/*by Daniel Jolley-Rogers u7511912
attempts to make an object fully connected*/
void makeObjectConnected(Object &object){
    //tries to make object connected by deleting vertices which are not part of a face
    auto toDelete = remove_if(
        object.vertices.begin(), object.vertices.end(), [](shared_ptr<Vertex> vertex) {return !vertex->halfEdge.lock();});
    object.vertices.erase(toDelete, object.vertices.end());
}

/*by Daniel Jolley-Rogers u7511912
checks whether a given object is manifold*/
bool objectManifold(const Object &object){
    //to check edge manifoldness, make sure their are exactly 2 faces on each edge.
    //first check to make sure only 2 faces share an edge
    vector<vector<int>> numEdgesBetweenVertices(object.vertices.size(), vector<int>(object.vertices.size(), 0));
    for(const shared_ptr<Face> &face : object.faces){
        shared_ptr<HalfEdge> current = face->halfEdge.lock();
        int startID = current->id;
        do{
            if(!current->vertex || !current->next || !current->next->vertex){
                break;
            }
            shared_ptr<HalfEdge> previous = current;
            current = current->next;
            int currentID = current->vertex->id;
            int previousID = previous->vertex->id;
            numEdgesBetweenVertices[previousID][currentID] += 1;
            numEdgesBetweenVertices[currentID][previousID] += 1;
            if(numEdgesBetweenVertices[previousID][currentID] > 2){
                return false;
            }
        }
        while(current->id != startID);
    }
    for(vector<int> start : numEdgesBetweenVertices){
        for(int connections : start){
            if (connections > 0 && connections != 2){
                return false;
            }
        }
    }
    vector<vector<int>> startingPoints(object.vertices.size(), vector<int>());
    //then check if there are any edges with no face
    for(const shared_ptr<HalfEdge> &halfEdge : object.halfEdges){
        if(!halfEdge->face){
            return false;
        }
        int id = halfEdge->vertex->id;
        startingPoints[id].push_back(halfEdge->id);
    }
    //check vertex manifoldness by exploring vertex fans
    int index = 0;
    int maxFan = object.vertices.size();

    for(const shared_ptr<Vertex> &vertex : object.vertices){
        vector<int> currentFan;
        shared_ptr<HalfEdge> halfEdge = vertex->halfEdge.lock();
        shared_ptr<HalfEdge> twin;
        if(!vertex || !halfEdge){
            continue;
        }
        //get a halfedge on a random fan of the current vertex
        //get all halfedges which are part of the fan by traverseing it
        //first forward
        bool fullyExplored = false;
        while(true){
            if(find(currentFan.begin(), currentFan.end(), halfEdge->id) != currentFan.end()){
                fullyExplored = true;
                break;
            }
            currentFan.push_back(halfEdge->id);
            if(currentFan.size() > maxFan){
                return false;
            }
            twin = halfEdge->twin.lock();
            if(!twin || !twin->face || !twin->next){
                break;
            }
            halfEdge = twin->next;
        }

        //then backwards
        halfEdge = vertex->halfEdge.lock()->previous.lock();
        twin = halfEdge->twin.lock();
        if(!fullyExplored && twin && twin->face){
            if(!twin){
                break;
            }
            halfEdge = twin;
            while(true){
                if(find(currentFan.begin(), currentFan.end(), halfEdge->id) != currentFan.end()){
                    break;
                }
                currentFan.push_back(halfEdge->id);
                if(currentFan.size() > maxFan){
                    return false;
                }
                halfEdge = halfEdge->previous.lock();
                twin = halfEdge->twin.lock();
                if(!twin || !twin->face || !twin->next){
                    break;
                }
                halfEdge = twin;
            }
        }
        //if there is a half edge coming from this vertex
        //which has not been found in the fan traversal
        //then the vertex is not manifold
        for(int id : startingPoints[vertex->id]){
            if(find(currentFan.begin(), currentFan.end(), id) == currentFan.end()){
                return false;
            }
        }
    }
    return true;
}

/*by Daniel Jolley-Rogers u7511912
attempts to make an object pseudo manifold*/
void makeObjectManifold(Object &object){
    //makes object psuedo manifold by connecting vertex fans even if there is a gap between them
    //this allows functions like Face::getHalfEdges() to work
    vector<vector<shared_ptr<HalfEdge>>> startingPoints(object.vertices.size());
    for(const shared_ptr<HalfEdge> &halfEdge : object.halfEdges){
        int id = halfEdge->vertex->id;
        startingPoints[id].push_back(halfEdge);
    }
    for(const vector<shared_ptr<HalfEdge>> &halfEdges : startingPoints){
        vector<vector<shared_ptr<HalfEdge>>> fans;
        set<int> inFan;
        //get all the fans that come from this vertex
        for(const shared_ptr<HalfEdge> &halfEdge : halfEdges){
            if(inFan.count(halfEdge->id) != 0){
                continue;
            }
            vector<shared_ptr<HalfEdge>> currentFan;
            shared_ptr<HalfEdge> currentHalfEdge = halfEdge;
            shared_ptr<HalfEdge> twin;
            //get all halfedges which are part of this fan by traverseing it
            //first forward
            bool fullyExplored = false;
            while(true){
                if(inFan.count(currentHalfEdge->id) != 0){
                    fullyExplored = true;
                    break;
                }
                currentFan.push_back(currentHalfEdge);
                inFan.insert(currentHalfEdge->id);
                twin = currentHalfEdge->twin.lock();
                if(!twin || !twin->face || !twin->next){
                    break;
                }
                currentHalfEdge = twin->next;
            }

            //then backwards
            currentHalfEdge = halfEdge->previous.lock();
            if(currentHalfEdge){
                twin = currentHalfEdge->twin.lock();
                if(!fullyExplored && twin && twin->face){
                    currentHalfEdge = twin;
                    while(true){
                        if(inFan.count(currentHalfEdge->id) != 0){
                            break;
                        }
                        currentFan.insert(currentFan.begin(), currentHalfEdge);
                        inFan.insert(currentHalfEdge->id);
                        currentHalfEdge = currentHalfEdge->previous.lock();
                        if(!currentHalfEdge){
                            break;
                        }
                        twin = currentHalfEdge->twin.lock();
                        if(!twin || !twin->face || !twin->next){
                            break;
                        }
                        currentHalfEdge = twin;
                    }
                }
            }
            fans.push_back(currentFan);
        }
        //skip if already one fan
        if (fans.size() <= 1){
            continue;
        }
        //connect fans together
        for(int i = 0; i < fans.size(); ++i){
            shared_ptr<HalfEdge> towardsVertex;
            shared_ptr<HalfEdge> awayFromVertex;
            if(i == fans.size() - 1){
                towardsVertex = fans[0].front()->previous.lock();
                awayFromVertex = fans[i].back();
                
            }
            else{
                towardsVertex = fans[i + 1].front()->previous.lock();
                awayFromVertex = fans[i].back();
            }
            if(awayFromVertex){
                awayFromVertex->twin = towardsVertex;
            }
            if(towardsVertex){
                towardsVertex->twin = awayFromVertex;
            }
        }
    }
}

/*by Daniel Jolley-Rogers u7511912
checks whether a given object has consisten facing*/
bool objectFacesConsistent(const Object &object){
    //two adjacent polygons have a consistant orientation if touching edges face opposite directions
    //check two ways, first by comparing winding direction, and secondly by comparing normals
    //make a bool for each origin vertex - destination vertex combo and set it to false
    set<vector<int>> originToDest;
    for(const shared_ptr<HalfEdge> &halfEdge : object.halfEdges){
        int origin = halfEdge->vertex->id;
        int destination = halfEdge->next->vertex->id;
        //if the current half edge's origin and destination have been encountered return false
        if(originToDest.find({origin, destination}) != originToDest.end()){
            return false;
        }
        //set the current origin and destination combo to true
        originToDest.insert({origin, destination});
        //check normal against neighbour
        if(!halfEdge->face){
            continue;
        }
        Vector3f currentNormal = halfEdge->face->normal;
        shared_ptr<HalfEdge> twin = halfEdge->twin.lock();
        if(!twin){
            continue;
        }
        Vector3f twinNormal = twin->face->normal;
        if(currentNormal.dot(twinNormal) < -1 + FLT_EPSILON){
            return false;
        }
    }
    return true;
}

/*by Daniel Jolley-Rogers u7511912
attempts to make an object's faces consistently faced*/
void makeObjectFacesConsistent(Object &object){
    struct Edge{
        int vertex1;
        int vertex2;
        Edge(int _v1, int _v2): vertex1(_v1), vertex2(_v2){}
        bool operator<(const Edge &other) const{
            return vertex1 < other.vertex1 || (vertex1 == other.vertex1 && vertex2 < other.vertex2);
        }
        size_t operator()() const noexcept {
            size_t h1 = std::hash<int>{}(vertex1);
            size_t h2 = std::hash<int>{}(vertex2);
            return h1 ^ (h2 << 1);
          }
    };
    vector<shared_ptr<Face>> consistentFaces;
    vector<shared_ptr<HalfEdge>> consistentHalfEdges;
    consistentFaces.reserve(object.faces.size());
    consistentHalfEdges.reserve(object.halfEdges.size());
    set<int> visited;

    map<Edge, vector<shared_ptr<Face>>> edgeToFaces;
    vector<shared_ptr<Face>> faces = object.faces;
    for(shared_ptr<Face> face : object.faces){
        shared_ptr<HalfEdge> halfEdge = face->halfEdge.lock();
        int startID = halfEdge->id;
        do{

            int v1 = halfEdge->vertex->id;
            int v2 = halfEdge->next->vertex->id;
            if(v1 > v2){
                swap(v1, v2);
            }
            edgeToFaces[Edge(v1, v2)].push_back(face);
            halfEdge = halfEdge->next;
        }
        while(halfEdge->id != startID);
    }

    stack<pair<shared_ptr<Face>, bool>> exploreFacesStack;
    //handle disconnected faces by starting in multiple spots
    //(faces will only be consistent with faces they are connected to)
    for(const shared_ptr<Face> &face : faces){
        if(visited.count(face->id) == 0){
            exploreFacesStack.push({face, false});
        }
        while(!exploreFacesStack.empty()){
            pair<shared_ptr<Face>, bool> pair = exploreFacesStack.top();
            exploreFacesStack.pop();
            bool reverse = pair.second;
            shared_ptr<Face> currentFace = pair.first;

            //directions - [from, to]
            vector<Edge> directions; 
            vector<shared_ptr<HalfEdge>> clones;
            shared_ptr<HalfEdge> currentHalfEdge = currentFace->halfEdge.lock();
            int startID = currentHalfEdge->id;
            do {
                shared_ptr<HalfEdge> newHalfEdge = make_shared<HalfEdge>();
                newHalfEdge->id = currentHalfEdge->id;
                newHalfEdge->face = currentHalfEdge->face;
                newHalfEdge->vertex = currentHalfEdge->vertex;
                clones.push_back(newHalfEdge);
                consistentHalfEdges.push_back(newHalfEdge);
                currentHalfEdge = currentHalfEdge->next;
            } 
            while (currentHalfEdge->id != startID);
    
            int n = clones.size();
            for(int i = 0; i < n; ++i){
                if(reverse){
                    clones[i]->next = clones[(i - 1 + n) % n];
                    clones[i]->previous = clones[(i + 1 + n) % n];
                }
                else{
                    clones[i]->next = clones[(i + 1 + n) % n];
                    clones[i]->previous = clones[(i - 1 + n) % n];
                }
                directions.push_back(Edge(clones[i]->vertex->id, clones[i]->next->vertex->id));
            }
            currentFace->halfEdge = clones[0];
            visited.insert(currentFace->id);
            consistentFaces.push_back(currentFace);
            
            for(const Edge &directedEdge : directions){
                int from = directedEdge.vertex1;
                int to = directedEdge.vertex2;
                vector<shared_ptr<Face>> adjacent;
                if(from < to){
                    adjacent = edgeToFaces[Edge(from, to)];
                }
                else{
                    adjacent = edgeToFaces[Edge(to, from)];
                }
                
                for(const shared_ptr<Face> &adj : adjacent){
                    if(adj->id == currentFace->id){
                        continue;
                    }
                    if(visited.find(adj->id) == visited.end()){
                        shared_ptr<HalfEdge> currentHalfEdge = adj->halfEdge.lock();
                        int startID = currentHalfEdge->id;
                        do{
                            shared_ptr<HalfEdge> previous = currentHalfEdge;
                            currentHalfEdge = currentHalfEdge->next;
                            int adjFrom = previous->vertex->id;
                            int adjTo = currentHalfEdge->vertex->id;
                            if(from == adjFrom && to == adjTo){
                                exploreFacesStack.push({adj, true});
                                break;
                            }
                            else if(from == adjTo && to == adjFrom){
                                exploreFacesStack.push({adj, false});
                                break;
                            }
                        }
                        while(currentHalfEdge->id != startID);
                    }
                }
            }
        }
    }

    for(const auto &edgeFaces : edgeToFaces){
        //assume there are at most 2 entries in edgeFaces
        vector<shared_ptr<HalfEdge>> adjacentHalfEdges;
        int numIterations = 0;
        Edge edge = edgeFaces.first;
        for(const shared_ptr<Face> &face : edgeFaces.second){
            numIterations += 1;
            if(numIterations > 2){
                break;
            }
            shared_ptr<HalfEdge> current = face->halfEdge.lock();
            int startID = current->id;
            do{
                int from = current->vertex->id;
                int to = current->next->vertex->id;
                if((edge.vertex1 == from && edge.vertex2 == to) || (edge.vertex1 == to && edge.vertex2 == from)){
                    adjacentHalfEdges.push_back(current);
                    break;
                }
                current = current->next;
            }
            while(current->id != startID);

        }
        if(adjacentHalfEdges.size() == 2){
            adjacentHalfEdges[0]->twin = adjacentHalfEdges[1];
            adjacentHalfEdges[1]->twin = adjacentHalfEdges[0];
        }
    }
    object.faces = move(consistentFaces);
    object.halfEdges = move(consistentHalfEdges);
}

/*by Daniel Jolley-Rogers u7511912
checks whether a given object is a tri mesh*/
bool objectTri(const Object &object){
    for(int i = 0; i < object.faces.size(); ++i){
        if(object.faces[i]->getHalfEdges().size() != 3){
            return false;
        }
    }
    return true;
}

/*by Daniel Jolley-Rogers u7511912
checks whether a given object is a quad mesh*/
bool objectQuad(const Object &object){
    for(int i = 0; i < object.faces.size(); ++i){
        if(object.faces[i]->getHalfEdges().size() != 4){
            return false;
        }
    }
    return true;
}

/*by Daniel Jolley-Rogers u7511912
code to make an object a tri mesh*/
namespace seidel{
    /*by Daniel Jolley-Rogers u7511912
    stores information about a specific point*/
    struct Point{
        Vector3f position;
        Vector3f colour;
        Vector2f textureCoords;
        Vector3f normal;
        Point(const shared_ptr<Vertex> &vert){
            position = vert->position;
            colour = vert->colour;
            textureCoords = vert->textureCoordinates;
            normal = vert->normal;
        }
        Point(Vector3f _position, Vector3f _colour, Vector2f _textureCoords, Vector3f _normal):
         position(_position), colour(_colour), textureCoords(_textureCoords), normal(_normal){}
        Point(){}
        bool operator==(const Point &other) const{
            auto lhs = tie(
                position.x(), position.y(), position.z(),
                colour.x(),   colour.y(),   colour.z(),
                textureCoords.x(), textureCoords.y(),
                normal.x(),   normal.y(),   normal.z());
            auto rhs = tie(
                other.position.x(), other.position.y(), other.position.z(),
                other.colour.x(),   other.colour.y(),   other.colour.z(),
                other.textureCoords.x(), other.textureCoords.y(),
                other.normal.x(),   other.normal.y(),   other.normal.z());
            return lhs == rhs;
        }
        bool operator<(const Point &other) const{
            auto lhs = tie(
                position.x(), position.y(), position.z(),
                colour.x(),   colour.y(),   colour.z(),
                textureCoords.x(), textureCoords.y(),
                normal.x(),   normal.y(),   normal.z());
            auto rhs = tie(
                other.position.x(), other.position.y(), other.position.z(),
                other.colour.x(),   other.colour.y(),   other.colour.z(),
                other.textureCoords.x(), other.textureCoords.y(),
                other.normal.x(),   other.normal.y(),   other.normal.z());
            return lhs < rhs;
        }
    };

    /*by Daniel Jolley-Rogers u7511912
    returns the rotated position of a vertex with a given index*/
    Eigen::Vector3f getValue(int key,  std::map<int, Eigen::Vector3f> &rotatedPos){
        auto it = rotatedPos.find(key);
        assert(it != rotatedPos.end());
        return it->second;
    }

    /*by Daniel Jolley-Rogers u7511912
    decomposes a given quadrilateral into triangles, handles concave and self intersecting quadrilaterals*/
    vector<vector<Point>> triangulateQuad(Point A, Point B, Point C, Point D){
        auto concave = [&]() ->pair<int, int>{
            Vector3f AB = (B.position - A.position).normalized();
            Vector3f BC = (C.position - B.position).normalized();
            Vector3f CD = (D.position - C.position).normalized();
            Vector3f DA = (A.position - D.position).normalized();
            //first check if convex CDA
            //check if cross product of sides is the same
            Vector3f base = DA.cross(AB);
            vector<int> direction(4);
            direction[0] = base.dot(DA.cross(AB));
            direction[1] = base.dot(AB.cross(BC));
            direction[2] = base.dot(BC.cross(CD));
            direction[3] = base.dot(CD.cross(DA));
            int numNeg;
            for(int i = 0; i < 4; ++i){
                if(direction[i] < 0){
                    numNeg += 1;
                }
            }
            if(numNeg == 2){
                return {2, -1};
            }
            else if(numNeg == 1){
                for(int i = 0; i < 4; ++i){
                    if(direction[i] < 0){
                        return {1, i};
                    }
                }
            }
            else if(numNeg == 3){
                for(int i = 0; i < 4; ++i){
                    if(!(direction[i] < 0)){
                        return {1, i};
                    }
                }
            }
            return {0, -1};
        };
        
        //check if already a triangle
        if(A == B){
            return {{B, C, D}};
        }
        else if(A == C){
            return {{B, C, D}};
        }
        else if(A == D){
            return {{A, B, C}};
        }
        else if(B == C){
            return {{D, A, B}};
        }
        else if(B == D){
            return {{A, B, C}};
        }
        else if(C == D){
            return {{A, B, C}};
        }
        pair<int, int> pair = concave();
        int isConcave = pair.first;
        int concavePoint = pair.second;
        if(isConcave == 0 || (isConcave == 1 && (concavePoint == 1 || concavePoint == 3))){
            return {{A, B, C}, {A, C, D}};
        }
        else if(isConcave == 1){
            return {{A, B, D}, {B, C, D}};
        }
        else{
            //if convex along both diagnols then the quad must self intersect
            //need to find coords where AC intersects BD
            //set A + (AB * n) = B + (bc * m)
            //and then solve for n and m
            Vector3f ad = D.position - A.position;
            Vector3f bc = C.position - B.position;
            Vector3f ab = B.position - A.position;
            Vector3f dc = C.position - B.position;

            float det = (ad.x() * (-bc.y())) - (ad.y() * (-bc.x()));
            
            //using cramer's rule
            float n = ((ab.x() * (-bc.y())) - (ab.y() * (-bc.x()))) / det;
            //if ad and bc are not parrallel and there intersection is withing the polygon then they are ones that self intersect
            if(ad.dot(bc) < 1 + FLT_EPSILON && n > -FLT_EPSILON && n < 1 + FLT_EPSILON){
                //finally get intersection
                Vector3f iPosition = A.position + (ad * n);
                Vector3f iColour = (A.colour * (1 - n)) + (C.colour * n);
                Vector2f iTextureCoords = (A.textureCoords * (1 - n)) + (C.textureCoords * n);
                Vector3f iNormal = (A.normal * (1 - n)) + (C.normal * n);
                Point I = Point(iPosition, iColour, iTextureCoords, iNormal);

                return{{A, B, I}, {C, D, I}};
            }
            //else the intersection must be on AB, DC
            else{
                float det = (ab.y() * (-dc.x())) - (ab.x() * (-dc.y()));
                
                //using cramer's rule
                float n = ((ad.x() * (-dc.y())) - (ad.y() * (-dc.y()))) / det;

                Vector3f iPosition = A.position + (ab * n);
                Vector3f iColour = (A.colour * (1 - n)) + (C.colour * n);
                Vector2f iTextureCoords = (A.textureCoords * (1 - n)) + (C.textureCoords * n);
                Vector3f iNormal = (A.normal * (1 - n)) + (C.normal * n);
                Point I = Point(iPosition, iColour, iTextureCoords, iNormal);

                return{{D, A, I}, {B, C, I}};
            }
        }
    }    

    /*by Daniel Jolley-Rogers u7511912
    helper function to handle tie breaks when y or x values match*/
    Vector3f tieBreak(const Vector3f a, const Vector3f b){
        if(a.y() > b.y()){
            return a;
        }
        else if(a.y() < b.y()){
            return b;
        }
        else{
            if(a.x() > b.x()){
                return a;
            }
            else if(a.x() < b.x()){
                return b;
            }
            else{
                if(a.z() > b.z()){
                    return a;
                }
                else{
                    return b;
                }
            }
        }
    }
    
    /*by Daniel Jolley-Rogers u7511912
    adds a vertex node into the Siedel tree*/
    shared_ptr<Node> processVertexNode(const shared_ptr<Vertex> &vert, shared_ptr<Node> &tree, vector<shared_ptr<HalfEdge>> faceEdges, int &nodeID, map<int, Vector3f> &rotatedPos){
        if(tree->nodeType == Node::ROOT){
            //nodeType only root when tree is empty
            float ray = rotatedPos.at(vert->id).y();
            tree->nodeType = Node::VERTEX;
            tree->vertex = vert;
    
            tree->left = make_shared<Node>(Node(nodeID++));
            tree->left->parent = tree;
            std::shared_ptr<Trapezoid> bottomTrap = make_shared<Trapezoid>(tree->left, -FLT_MAX, ray);
            tree->left->nodeType = Node::TRAPAZOID;
            tree->left->trapezoid = bottomTrap;
    
            tree->right = make_shared<Node>(nodeID++);
            tree->right->parent = tree;
            std::shared_ptr<Trapezoid> topTrap = make_shared<Trapezoid>(tree->right, ray, FLT_MAX);
            tree->right->nodeType = Node::TRAPAZOID;
            tree->right->trapezoid = topTrap;
            topTrap->down.push_back(bottomTrap);
            bottomTrap->up.push_back(topTrap);
            return tree;
        }
        shared_ptr<Node> currentNode = tree;
        while(true){
            if(currentNode->nodeType == Node::VERTEX){
                Vector3f a = getValue(vert->id, rotatedPos);
                Vector3f b = getValue(currentNode->vertex.lock()->id, rotatedPos);
                if(a == b){
                    return currentNode;
                }
                else if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                    currentNode = currentNode->right;
                }
                else{
                    currentNode = currentNode->left;
                }
            }
            else if(currentNode->nodeType == Node::EDGE){
                //need to interpolate to find which side of an edge a vertex is
                shared_ptr<HalfEdge> e = currentNode->edge;
                Vector3f origin = getValue(e->vertex->id, rotatedPos);
                Vector3f dest = getValue(e->next->vertex->id, rotatedPos);
                float yValue = rotatedPos[vert->id].y();
                Vector3f unitVector = (dest - origin).normalized();
                float mult = (yValue - origin.y()) / unitVector.y();
                Vector3f intercept = origin + (unitVector * mult);
                Vector3f a = getValue(vert->id, rotatedPos);
                Vector3f b = intercept;
                    if(a.x() > b.x() || (a.x() == b.x() && tieBreak(a, b) == a)){
                        currentNode = currentNode->right;
                    }
                    else{
                        currentNode = currentNode->left;
                    }
            }
            else{
                //if not vertex or edge must be trapezoid
                shared_ptr<Trapezoid> old = currentNode->trapezoid;
                float ray = rotatedPos.at(vert->id).y();
                float oldHighRay = old->highRay;
                float oldLowRay = old->lowRay;
                currentNode->nodeType = Node::VERTEX;
                currentNode->trapezoid = nullptr;
                currentNode->vertex = vert;
                currentNode->left = make_shared<Node>(nodeID++);
                currentNode->left->parent = currentNode;
                std::shared_ptr<Trapezoid> bottomTrap = make_shared<Trapezoid>(currentNode->left, oldLowRay, ray);
                currentNode->left->nodeType = Node::TRAPAZOID;
                currentNode->left->trapezoid = bottomTrap;
                currentNode->right = make_shared<Node>(nodeID++);
                currentNode->right->parent = currentNode;
                std::shared_ptr<Trapezoid> topTrap = make_shared<Trapezoid>(currentNode->right, ray, oldHighRay);
                currentNode->right->nodeType = Node::TRAPAZOID;
                currentNode->right->trapezoid = topTrap;
                topTrap->up = old->up;
                bottomTrap->down = old->down;
                topTrap->down.push_back(bottomTrap);
                bottomTrap->up.push_back(topTrap);
                for(const shared_ptr<Trapezoid> &above : topTrap->up){
                    replace(above->down.begin(), above->down.end(), old, topTrap);
                }
                for(const shared_ptr<Trapezoid> &below : bottomTrap->down){
                    replace(below->up.begin(), below->up.end(), old, bottomTrap);
                }
                return currentNode;
            }
        }
    }
    
    /*by Daniel Jolley-Rogers u7511912
    inserts an edge into the seidel tree at a given node*/
    void insertEdge(const shared_ptr<Node> &node, const shared_ptr<HalfEdge> &halfEdge, int &nodeID, map<int, Vector3f> &rotatedPos){
        shared_ptr<Trapezoid> old = node->trapezoid;
        float highRay = old->highRay;
        float lowRay = old->lowRay;
        node->nodeType = Node::EDGE;
        node->trapezoid = nullptr;
        node->edge = halfEdge;
        node->left = make_shared<Node>(nodeID++);
        node->left->parent = node;
        std::shared_ptr<Trapezoid> bottomTrap = make_shared<Trapezoid>(node->left, lowRay, highRay);
        node->left->nodeType = Node::TRAPAZOID;
        node->left->trapezoid = bottomTrap;
        node->right = make_shared<Node>(Node(nodeID++));
        node->right->parent = node;
        std::shared_ptr<Trapezoid> topTrap = make_shared<Trapezoid>(node->right, lowRay, highRay);
        node->right->nodeType = Node::TRAPAZOID;
        node->right->trapezoid = topTrap;
        Vector3f a;
        Vector3f b;
        vector<shared_ptr<Node>> children = {node->left, node->right};
        //get new ups and downs, and set neighbours
        for(const shared_ptr<Trapezoid> &above : old->up){
            auto iter = find(above->down.begin(), above->down.end(), old);
            if ( iter != above->down.end() ) {
                above->down.erase(iter);
            }
            //get lower left above x coord
            float lowerLeftAbove;
            shared_ptr<HalfEdge> lseg = above->lseg;
            if(!lseg){
                lowerLeftAbove = -FLT_MAX;
            }
            else{
                a = getValue(lseg->vertex->id, rotatedPos);
                b = getValue(lseg->next->vertex->id, rotatedPos);
                if(a.y() < b.y() || (a.y() == b.y() && tieBreak(a, b) == b)){
                    lowerLeftAbove = a.x();
                }
                else{
                    lowerLeftAbove = b.x();
                }
            }
    
            //get lower right above x coord
            float lowerRightAbove;
            shared_ptr<HalfEdge> rseg = above->rseg;
            if(!rseg){
                lowerRightAbove = FLT_MAX;
            }
            else{
                a = getValue(rseg->vertex->id, rotatedPos);
                b = getValue(rseg->next->vertex->id, rotatedPos);
                if(a.y() < b.y() || (a.y() == b.y() && tieBreak(a, b) == b)){
                    lowerRightAbove = a.x();
                }
                else{
                    lowerRightAbove = b.x();
                }
            }
            for(const shared_ptr<Node> &child : children){
                //get upper left child x coord
                float upperLeftChild;
                lseg = child->trapezoid->lseg;
                if(!lseg){
                    upperLeftChild = -FLT_MAX;
                }
                else{
                    a = getValue(lseg->vertex->id, rotatedPos);
                    b = getValue(lseg->next->vertex->id, rotatedPos);
                    if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                        upperLeftChild = a.x();
                    }
                    else{
                        upperLeftChild = b.x();
                    }
                }
                //get upper right child x coord
                float upperRightChild;
                rseg = child->trapezoid->rseg;
                if(!rseg){
                    upperRightChild = FLT_MAX;
                }
                else{
                    a = getValue(rseg->vertex->id, rotatedPos);
                    b = getValue(rseg->next->vertex->id, rotatedPos);
                    if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                        upperRightChild = a.x();
                    }
                    else{
                        upperRightChild = b.x();
                    }
                }
                if(lowerLeftAbove < upperRightChild && lowerRightAbove > upperLeftChild){
                    above->down.push_back(child->trapezoid);
                }
            }
        }
        for(const shared_ptr<Trapezoid> &below : old->down){
            auto iter = find(below->up.begin(), below->up.end(), old);
            if ( iter != below->up.end() ) {
                below->up.erase(iter);
            }
            //get upper left below x coord
            float upperLeftBelow;
            shared_ptr<HalfEdge> lseg = below->lseg;
            if(!lseg){
                upperLeftBelow = -FLT_MAX;
            }
            else{
                a = getValue(lseg->vertex->id, rotatedPos);
                b = getValue(lseg->next->vertex->id, rotatedPos);
                if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                    upperLeftBelow = a.x();
                }
                else{
                    upperLeftBelow = b.x();
                }
            }
            //get upper right below x coord
            float upperRightBelow;
            shared_ptr<HalfEdge> rseg = below->rseg;
            if(!rseg){
                upperRightBelow = FLT_MAX;
            }
            else{
                a = getValue(rseg->vertex->id, rotatedPos);
                b = getValue(rseg->next->vertex->id, rotatedPos);
                if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                    upperRightBelow = a.x();
                }
                else{
                    upperRightBelow = b.x();
                }
            }
            for(const shared_ptr<Node> &child : children){
                //get lower left child x coord
                float lowerLeftChild;
                lseg = child->trapezoid->lseg;
                if(!lseg){
                    lowerLeftChild = -FLT_MAX;
                }
                else{
                    a = getValue(lseg->vertex->id, rotatedPos);
                    b = getValue(lseg->next->vertex->id, rotatedPos);
                    if(a.y() < b.y() || (a.y() == b.y() && tieBreak(a, b) == b)){
                        lowerLeftChild = a.x();
                    }
                    else{
                        lowerLeftChild = b.x();
                    }
                }
                //get lower right child x coord
                float lowerRightChild;
                rseg = child->trapezoid->rseg;
                if(!rseg){
                    lowerRightChild = FLT_MAX;
                }
                else{
                    a = getValue(rseg->vertex->id, rotatedPos);
                    b = getValue(rseg->next->vertex->id, rotatedPos);
                    if(a.y() < b.y() || (a.y() == b.y() && tieBreak(a, b) == b)){
                        lowerRightChild = a.x();
                    }
                    else{
                        lowerRightChild = b.x();
                    }
                }
                if(upperLeftBelow < lowerRightChild && upperRightBelow > lowerLeftChild){
                    below->up.push_back(child->trapezoid);
                }
            }
        }
    }
    
    /*by Daniel Jolley-Rogers u7511912
    adds an edge into the siedel tree*/
    void processEdgeNode(const shared_ptr<HalfEdge> &halfEdge, const shared_ptr<Node> &lower, const shared_ptr<Node> &upper, int &nodeID, map<int, Vector3f> &rotatedPos){
        int placeInserted;
        //insert edge at upper
        shared_ptr<Node> currentNode = upper->left;
        while(true){
            if(currentNode->nodeType == Node::VERTEX){
                currentNode = currentNode->right;
            }
            else if(currentNode->nodeType == Node::EDGE){
                //compare top vertices
                shared_ptr<Vertex> targetTop;
                Vector3f o = getValue(halfEdge->vertex->id, rotatedPos);
                Vector3f n = getValue(halfEdge->next->vertex->id, rotatedPos);
                if(o.y() > n.y() || (o.y() == n.y() && tieBreak(o, n) == o)){
                    targetTop = halfEdge->vertex;
                }
                else{
                    targetTop = halfEdge->next->vertex;
                }
                shared_ptr<Vertex> currentTop;
                shared_ptr<HalfEdge> edge = currentNode->edge;
                o = getValue(edge->vertex->id, rotatedPos);
                n = getValue(edge->next->vertex->id, rotatedPos);
                if(o.y() > n.y() || (o.y() == n.y() && tieBreak(o, n) == o)){
                    currentTop = edge->vertex;
                }
                else{
                    currentTop = edge->next->vertex;
                }
                Vector3f a = getValue(targetTop->id, rotatedPos);
                Vector3f b = getValue(currentTop->id, rotatedPos);
                if(a.x() > b.x() || (a.x() == b.x() && tieBreak(a, b) == a)){
                    currentNode = currentNode->right;
                }
                else{
                    currentNode = currentNode->left;
                }
            }
            else{
                placeInserted = currentNode->id;
                insertEdge(currentNode, halfEdge, nodeID, rotatedPos);
                break;
            }
        }
        //insert edge at lower
        currentNode = lower->right;
        while(true){
            if(currentNode->id == placeInserted){
                break;
            }
            if(currentNode->nodeType == Node::VERTEX){
                currentNode = currentNode->left;
            }
            else if(currentNode->nodeType == Node::EDGE){
                //compare bottom vertices
                shared_ptr<Vertex> targetBottom;
                Vector3f o = getValue(halfEdge->vertex->id, rotatedPos);
                Vector3f n = getValue(halfEdge->next->vertex->id, rotatedPos);
                if(o.y() < n.y() || (o.y() == n.y() && tieBreak(o, n) == n)){
                    targetBottom = halfEdge->vertex;
                }
                else{
                    targetBottom = halfEdge->next->vertex;
                }
                shared_ptr<Vertex> currentBottom;
                shared_ptr<HalfEdge> edge = currentNode->edge;
                o = getValue(edge->vertex->id, rotatedPos);
                n = getValue(edge->next->vertex->id, rotatedPos);
                if(o.y() < n.y() || (o.y() == n.y() && tieBreak(o, n) == n)){
                    currentBottom = edge->vertex;
                }
                else{
                    currentBottom = edge->next->vertex;
                }
                Vector3f a = getValue(targetBottom->id, rotatedPos);
                Vector3f b = getValue(currentBottom->id, rotatedPos);
                if(a.x() > b.x() || (a.x() == b.x() && tieBreak(a, b) == a)){
                    currentNode = currentNode->right;
                }
                else{
                    currentNode = currentNode->left;
                }
            }
            else{
                insertEdge(currentNode, halfEdge, nodeID, rotatedPos);
                break;
            }
        }
    }

    /*by Daniel Jolley-Rogers u7511912
    finds the point where a ray intersects an edge and returns it 
    with all relavent position, texture, normal and colour information*/
    inline Point yInterceptFinder(const shared_ptr<Vertex> origin, const shared_ptr<Vertex> dest, float yIntercept, bool fromPX, map<int, Vector3f> &rotatedPos){
        Vector3f rotatedOrigin = getValue(origin->id, rotatedPos);
        Vector3f rotatedDest = getValue(dest->id, rotatedPos);
        Vector3f edgeVector = rotatedDest - rotatedOrigin;
        Vector3f position;
        Vector3f colour;
        Vector2f textureCoords;
        Vector3f normal;
        const float eps = 1e-6f * edgeVector.norm();  
        if (fabs(edgeVector.y()) < eps) {
            if(fromPX){
                if(rotatedOrigin.x() < rotatedDest.x()){
                    position = origin->position;
                    colour = origin->colour;
                    textureCoords = origin->textureCoordinates;
                    normal = origin->normal;
                }
                else{
                    position = dest->position;
                    colour = dest->colour;
                    textureCoords = dest->textureCoordinates;
                    normal = dest->normal;
                }
            }
            else{
                if(rotatedOrigin.x() > rotatedDest.x()){
                    position = origin->position;
                    colour = origin->colour;
                    textureCoords = origin->textureCoordinates;
                    normal = origin->normal;
                }
                else{
                    position = dest->position;
                    colour = dest->colour;
                    textureCoords = dest->textureCoordinates;
                    normal = dest->normal;
                }
            }
        }
        else{
            //in rotated coords
            float mult = (yIntercept - rotatedOrigin.y()) / (edgeVector.y());
            mult = clamp(mult, 0.0f, 1.0f);//then clamp to [0,1]
            //in world coords
            position = origin->position + (mult * (dest->position - origin->position));
            colour = (origin->colour * (1 - mult)) + (dest->colour * mult);
            textureCoords = (origin->textureCoordinates * (1 - mult)) + (dest->textureCoordinates * mult);
            normal = (origin->normal * (1 - mult)) + (dest->normal * mult);
        }
        return Point(position, colour, textureCoords, normal);
    }

    /*by Daniel Jolley-Rogers u7511912
    the main seidel function, builds a tree then triangulates all the trapezoids in that tree*/
    vector<vector<Point>> seidel(const shared_ptr<Face> &face, Matrix4f rotationMatrix){
         //Seidel's algorithm for decomposing to trapezoids (quadralaterials)
        vector<vector<Point>> triangles;
        int nodeID = 0;
        //I use SEIDEL'S algorithm to perform trapezoidation
        //source: http://www.polygontriangulation.com/2018/07/triangulation-algorithm.html

        vector<shared_ptr<HalfEdge>> faceEdges = face->getHalfEdges();
        //randomise edge order
        unsigned seed = chrono::system_clock::now().time_since_epoch().count();
        map<int, Vector3f> rotatedPos;
        shuffle(faceEdges.begin(), faceEdges.end(), default_random_engine(seed));
        for(const shared_ptr<HalfEdge> &halfEdge : faceEdges){
            Vector4f position4f;
            position4f.head(3) = halfEdge->vertex->position;
            position4f[3] = 1;
            position4f = rotationMatrix * position4f;
            rotatedPos[halfEdge->vertex->id] = position4f.head(3);
        }
        shared_ptr<Node> tree = make_shared<Node>(nodeID++);
        tree->nodeType = Node::ROOT;
        //get arbitrary edge from list of edges
        for(const shared_ptr<HalfEdge> &edge : faceEdges){
            //get edge vertices
            shared_ptr<Vertex> v1 = edge->vertex;
            shared_ptr<Vertex> v2 = edge->next->vertex;
            //find higher and lower vertex
            Vector3f a = getValue(v1->id, rotatedPos);
            Vector3f b = getValue(v2->id, rotatedPos);
            shared_ptr<Vertex> higher;
            shared_ptr<Vertex> lower;
            if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                higher = v1;
                lower = v2;
            }
            else{
                higher = v2;
                lower = v1;
            }
            //traverse tree and place higher vertex and lower vertex
            shared_ptr<Node> upperNode = processVertexNode(higher, tree, faceEdges, nodeID, rotatedPos);
            shared_ptr<Node> lowerNode = processVertexNode(lower, tree, faceEdges, nodeID, rotatedPos);
            //place edge below upper node but above lower node
            processEdgeNode(edge, lowerNode, upperNode, nodeID, rotatedPos);
        }
        //get list of trapezoids and make sure their edges are finalised
        //and set their validatiy
        vector<shared_ptr<Trapezoid>> trapezoids;
        stack<shared_ptr<Node>> nodeStack;
        nodeStack.push(tree);
        while(!nodeStack.empty()) {
            shared_ptr<Node> node = nodeStack.top(); 
            nodeStack.pop();
            if(node->nodeType == Node::TRAPAZOID){
                node->trapezoid->setEdges(node);
                node->trapezoid->inPolygon();
                trapezoids.push_back(node->trapezoid);
            }
            else{
                nodeStack.push(node->left);
                nodeStack.push(node->right);
            }
        }
        //transform trapezoids into triangles
        for(const shared_ptr<Trapezoid> &trap : trapezoids){
            if(!trap->lowRay || !trap->highRay || !trap->lseg || !trap->rseg){
                continue;
            }
            //get vertices of trapezoid
            //use rays and side segments to interporlate the four vertices
            float highY = trap->highRay;
            float lowY = trap->lowRay;
            shared_ptr<HalfEdge> lseg = trap->lseg;
            shared_ptr<HalfEdge> rseg = trap->rseg;
            shared_ptr<Vertex> leftO = lseg->vertex;
            shared_ptr<Vertex> leftD = lseg->next->vertex;
            shared_ptr<Vertex> rightO = rseg->vertex;
            shared_ptr<Vertex> rightD = rseg->next->vertex;
            
            Point A = yInterceptFinder(leftO, leftD, lowY, true, rotatedPos);
            Point B = yInterceptFinder(rightO, rightD, lowY, false, rotatedPos);
            Point C = yInterceptFinder(rightO, rightD, highY, false, rotatedPos);
            Point D = yInterceptFinder(leftO, leftD, highY, true, rotatedPos);

            vector<vector<Point>> currentTriangles = triangulateQuad(A, B, C, D);
            triangles.insert(triangles.end(), currentTriangles.begin(), currentTriangles.end());
        }
        return triangles;
    }

    /*by Daniel Jolley-Rogers u7511912
    stores information about a planar segment of a polygon*/
    struct PlaneSegment{
        Vector3f normal;
        Vector3f planePoint;
        shared_ptr<HalfEdge> start;
        shared_ptr<HalfEdge> End;
    };

    /*by Daniel Jolley-Rogers u7511912
    checks whether a given polygon is planar either returns the whole polygon if it is
    or one planar segment of the polygon if it isn't*/
    bool facePlanar(PlaneSegment &plane, const shared_ptr<HalfEdge> &halfEdge, set<int> &visited){
        shared_ptr<HalfEdge> adjacentHalfEdge;
        Vector3f currentVector = halfEdge->next->vertex->position - halfEdge->vertex->position;
        Vector3f planePoint = halfEdge->vertex->position;
        Vector3f adjacentVector, normal;
        adjacentHalfEdge = halfEdge->next;
        bool checkReverse = false;
        bool faceLine = true;
        //get closest halfedge that hasn't already been visited and is not colinear
        //first look forwards
        do{
            if(visited.count(adjacentHalfEdge->id) == 0){
                //get the equation for the current plane using the half edges as vectors
                adjacentVector = adjacentHalfEdge->next->vertex->position - adjacentHalfEdge->vertex->position;
                normal = currentVector.cross(adjacentVector).normalized();
                if(normal.squaredNorm() > FLT_EPSILON * FLT_EPSILON){
                    faceLine = false;
                    break;
                }
                adjacentHalfEdge = adjacentHalfEdge->next;
            }
            else{
                checkReverse = true;
                break;
            }
        }while(adjacentHalfEdge->id != halfEdge->id);
        //then if havn't found good halfEdge look backwards
        if(checkReverse){
            adjacentHalfEdge = halfEdge->previous.lock();
            do{
                if(visited.count(adjacentHalfEdge->id) == 0){
                    adjacentVector = adjacentHalfEdge->next->vertex->position - adjacentHalfEdge->vertex->position;
                    normal = adjacentVector.cross(currentVector).normalized();
                    if(normal.squaredNorm() > FLT_EPSILON * FLT_EPSILON){
                        faceLine = false;
                        break;
                    }
                    adjacentHalfEdge = adjacentHalfEdge->previous.lock();
                }
                else{
                    throw(string("no way to make face planar"));
                }
            }while(adjacentHalfEdge->id != halfEdge->id);
        }
        //deal with case where face is close to being a line
        if(faceLine){
            plane.End = halfEdge;
            plane.start = halfEdge;
            plane.planePoint = halfEdge->vertex->position;
            Vector3f line = halfEdge->next->vertex->position - halfEdge->vertex->position;
            if(line.cross(Vector3f(1, 0, 0)).squaredNorm() > FLT_EPSILON * FLT_EPSILON){
                plane.normal = line.cross(Vector3f(1, 0, 0)).normalized();
            }
            else{
                plane.normal = line.cross(Vector3f(0, 1, 0)).normalized();
            }
            return true;
        }

        //now that we have the plane we need to find all the start and ends points of the segment
        shared_ptr<HalfEdge> start;
        shared_ptr<HalfEdge> end;
        shared_ptr<HalfEdge> current = halfEdge;
        int currentID = current->id;
        bool fullLoop = true;
        //first find the end of this planar section if it exists
        do{
            end = current;
            Vector3f nextPos = current->next->next->vertex->position;
            if(fabs((nextPos - planePoint).dot(normal)) > FLT_EPSILON){
                fullLoop = false;
                break;
            }
            current = current->next;
        }while(currentID != current->id);
        if(!fullLoop){
             //then if the section is not closed find the start of this planar section
             current = halfEdge;
             do{
                start = current;
                shared_ptr<HalfEdge> previous = current->previous.lock();
                Vector3f previousPos = previous->vertex->position;
                if(fabs((previousPos - planePoint).dot(normal)) > FLT_EPSILON){
                    break;
                }
                current = previous;
             }while(currentID != current->id);
        }
        else{
            start = end;
        }
        plane.End = end;
        plane.start = start;
        plane.normal = normal;
        plane.planePoint = halfEdge->vertex->position;
        if(start->id == end->id){
            return true;
        }
        else{
            return false;
        }
    }

    /*by Daniel Jolley-Rogers u7511912
    decomposes a polygon into planar segments*/
    vector<shared_ptr<Face>> planarDecompose(const shared_ptr<Face> &face, int &oldFaceID){
        //breaks a face into its planar segments
        auto enforceWinding = [](const shared_ptr<Face> &currentFace, const Vector3f& targetNormal) ->void{
            // gather vertices in cycle order
            vector<Vector3f> points;
            shared_ptr<HalfEdge> halfEdge = currentFace->halfEdge.lock();
            shared_ptr<HalfEdge> currentHalfEdge  = halfEdge;
            vector<shared_ptr<HalfEdge>> halfEdges;
            do {
                points.push_back(currentHalfEdge->vertex->position);
                halfEdges.push_back(currentHalfEdge);
                currentHalfEdge = currentHalfEdge->next;
            } while(currentHalfEdge != halfEdge);
            // compute its current normal
            Vector3f currentNormal = (points[1] - points[0]).cross(points[2] - points[0]);
            if(currentNormal.dot(targetNormal) < 0){
                // reverse the cycle: swap next/previous on each halfEdge
                currentHalfEdge = halfEdge;
                int n = halfEdges.size();
                for(int i = 0; i < n; ++i){
                    halfEdges[i]->next = halfEdges[(i - 1 + n) % n];
                    halfEdges[i]->previous = halfEdges[(i + 1 + n) % n];
                }
                currentFace->halfEdge = halfEdges[0];
            }
        };

        //first check if face is planar
        vector<shared_ptr<Vertex>> vertices = face->getVertices();
        Vector3f pNormal = (vertices[1]->position - vertices[0]->position).cross(vertices[2]->position - vertices[0]->position);
        bool planar = true;
        for(int i = 3; i < vertices.size(); ++i){
            if((vertices[i]->position - vertices[0]->position).dot(pNormal) > FLT_EPSILON){
                planar = false;
                break;
            }
        }
        //if face is already planar just return the face
        PlaneSegment t;
        set<int> visited; 
        if(facePlanar(t, face->halfEdge.lock(), visited)){
            return {face};
        }
        //get max halfEdgeid number
        int halfEdgeID = 0;
        for(const shared_ptr<HalfEdge> &halfEdge : face->getHalfEdges()){
            int tempID = halfEdge->id;
            if(tempID > halfEdgeID){
                halfEdgeID = tempID;
            }
        }
        halfEdgeID += 1;
        stack<shared_ptr<HalfEdge>> halfEdgeStack;
        halfEdgeStack.push(face->halfEdge.lock());
        vector<shared_ptr<Face>> planarFaces;
        while(!halfEdgeStack.empty()){
            shared_ptr<HalfEdge> currentHalfEdge = halfEdgeStack.top();
            halfEdgeStack.pop();
            //if already done this halfEdge continue
            if(visited.count(currentHalfEdge->id) != 0){
                continue;
            }
            visited.insert(currentHalfEdge->id); 
            PlaneSegment currentPlane;
            bool restPlanar = facePlanar(currentPlane, currentHalfEdge, visited);
                       
            if(!restPlanar){
                //then if the section is not closed find the start of this planar section
                shared_ptr<HalfEdge> remainderStart = currentPlane.End->next;
                shared_ptr<HalfEdge> remainderEnd = currentPlane.start->previous.lock();
                //close new face and old face
                shared_ptr<HalfEdge> newPlanarHalfEdge = make_shared<HalfEdge>(HalfEdge(halfEdgeID++));
                shared_ptr<HalfEdge> newRemainderHalfEdge = make_shared<HalfEdge>(HalfEdge(halfEdgeID++));
                //make planar half edge
                newPlanarHalfEdge->previous = currentPlane.End;
                newPlanarHalfEdge->next = currentPlane.start;
                newPlanarHalfEdge->vertex = remainderStart->vertex;
                currentPlane.End->next = newPlanarHalfEdge;
                currentPlane.start->previous = newPlanarHalfEdge;
                newPlanarHalfEdge->twin = newRemainderHalfEdge;
                //make half edge for remainder of old face
                newRemainderHalfEdge->previous = remainderEnd;
                newRemainderHalfEdge->next = remainderStart;
                newRemainderHalfEdge->vertex = currentPlane.start->vertex;
                remainderEnd->next = newRemainderHalfEdge;
                remainderStart->previous = newRemainderHalfEdge;
                newRemainderHalfEdge->twin = newPlanarHalfEdge;

                //add remainder start and remainder end to stack
                halfEdgeStack.push(remainderStart);
                halfEdgeStack.push(remainderEnd);
            }
            shared_ptr<Face> planarFace = make_shared<Face>(oldFaceID++);
            planarFace->normal = currentPlane.normal;
            planarFace->halfEdge = currentHalfEdge;
            enforceWinding(planarFace, face->normal);
            shared_ptr<HalfEdge> current = currentPlane.start;
            int startID = current->id;
            do{
                current->face = planarFace;
                current = current->next;
            }while(current->id != startID);
            planarFaces.push_back(planarFace);
        }
        if (planarFaces.empty()) {
            // something went wrong—fall back to the un‐split face
            return { face };
        }
        return planarFaces;
    }

    /*by Daniel Jolley-Rogers u7511912
    returns the rotation needed to rotate a given face onto the x-y plane*/
    Matrix4f getRotation(const shared_ptr<Face> &face){
        Vector3f normal = face->normal.normalized();
        static const Vector3f desiredFacing = Vector3f(0, 0, -1);
        float dot = normal.dot(desiredFacing);
        Matrix4f rotation = Matrix4f::Identity();
        if (dot < 1.0f - FLT_EPSILON) {
            Quaternionf q = Quaternionf::FromTwoVectors(normal, desiredFacing);
            Matrix3f rotation3 = q.toRotationMatrix();
            rotation.block<3,3>(0,0) = rotation3;
        }
        // Already within epsilon of -Z, so just return identity.
        return rotation;
    }

    /*by Daniel Jolley-Rogers u7511912
    decomposes a given face into triangles, shortcuts to the end if the face is already a triangle or is a quad*/
    vector<vector<Point>> triangulate(const shared_ptr<Face> &face, int &oldFaceID){
        vector<vector<Point>> triangles;
        //check how many sides current face has
        shared_ptr<HalfEdge> halfEdge = face->halfEdge.lock();
        shared_ptr<HalfEdge> current = halfEdge;
        int startID = current->id;
        int numEdges = 0;
        do{
            current = current->next;
            numEdges += 1;
        }
        while(current->id != startID);
        //first break face into planar faces
        //if it is already a triangle, return unchanged
        current = face->halfEdge.lock();
        if(numEdges == 3){
            Point A = Point(current->vertex);
            Point B = Point(current->next->vertex);
            Point C = Point(current->next->next->vertex);
            triangles.push_back({A, B, C});
        }
        //if it is a quad use basic quad triangulation
        else if (numEdges == 4){
            Point A = Point(current->vertex);
            Point B = Point(current->next->vertex);
            Point C = Point(current->next->next->vertex);
            Point D = Point(current->next->next->next->vertex);

            vector<vector<Point>> temp = triangulateQuad(A, B, C, D);
            triangles.insert(triangles.end(), temp.begin(), temp.end());
        }
        //else use Seidel's
        else{
            for(const shared_ptr<Face> &planarFace : planarDecompose(face, oldFaceID)){
                Matrix4f rotationMatrix = getRotation(planarFace);
                vector<vector<Point>> temp = seidel(planarFace, rotationMatrix);
                triangles.insert(triangles.end(), temp.begin(), temp.end());
            }
        }
        vector<vector<Point>> goodTriangles;
        for(vector<Point> tri : triangles){
            if(tri[0].position == tri[1].position || tri[0].position == tri[2].position || tri[1].position == tri[2].position){
                continue;
            }
            goodTriangles.push_back(tri);
        }
        return goodTriangles;
    }
}

/*by Daniel Jolley-Rogers u7511912
attempts to make an object a tri mesh*/
void makeObjectTri(Object &object){
    vector<shared_ptr<Vertex>> vertices;
    vector<shared_ptr<Face>> faces;
    vector<shared_ptr<HalfEdge>> halfEdges;
    map<seidel::Point, int> vertexIndex;
    map<int, seidel::Point> indexVertex;
    int vertexID, faceID, halfEdgeID;
    vertexID = faceID = halfEdgeID = 0;
    map<int, vector<shared_ptr<HalfEdge>>> halfEdgesByDestination;

    ///get max Faceid number
    int oldFaceID = 0;
    for(const shared_ptr<Face> &face : object.faces){
        int tempID = face->id;
        if(tempID > oldFaceID){
            oldFaceID = tempID;
        }
    }
    oldFaceID += 1;

    auto makeFaceTri = [&](const shared_ptr<Face> face) -> void{
        vector<vector<seidel::Point>> faceTriangles = seidel::triangulate(face, oldFaceID);
        for(const vector<seidel::Point> &triangle : faceTriangles){
            for(const seidel::Point &point : triangle){
                if(vertexIndex.count(point) == 0){
                    vertexIndex.insert({point, vertexID});
                    vertexID += 1;
                }
            }
        }
        vertices.resize(vertexIndex.size());

        for(const vector<seidel::Point> &faceCoords : faceTriangles){
            shared_ptr<Face> face = make_shared<Face>(Face(faceID++));
            shared_ptr<HalfEdge> previous = nullptr;
            shared_ptr<Vertex> firstVertex;
            shared_ptr<HalfEdge> firstHalfEdge;
            for(const seidel::Point &point: faceCoords){
                int vertexNum =vertexIndex[point];
                shared_ptr<HalfEdge> halfEdge = make_shared<HalfEdge>(HalfEdge(halfEdgeID++));
                if(previous != nullptr){
                    previous->next = halfEdge;
                    halfEdge->previous = previous;
                    if (halfEdgesByDestination.find(vertexNum) == halfEdgesByDestination.end()){
                        halfEdgesByDestination.insert({vertexNum, vector<shared_ptr<HalfEdge>>()});
                    }
                    halfEdgesByDestination[vertexNum].push_back(previous);
                }
                else{
                    firstHalfEdge = halfEdge;
                }
                shared_ptr<Vertex> vertex;
                if(vertices[vertexNum] == nullptr){
                    vertex = make_shared<Vertex>(Vertex(vertexNum));
                    vertex->position = point.position;
                    vertex->colour = point.colour;
                    vertex->textureCoordinates = point.textureCoords;
                    vertex->normal = point.normal;
                    vertex->halfEdge = halfEdge;
                    vertices[vertexNum] = vertex;
                }
                else{
                    vertex = vertices[vertexNum];
                }
                halfEdge->vertex = vertex;
                halfEdge->face = face;
                halfEdges.push_back(halfEdge);
                previous = halfEdge;
            }
            previous->next = firstHalfEdge;
            firstHalfEdge->previous = previous;
            int id = firstHalfEdge->vertex->id;
            if (halfEdgesByDestination.find(id) == halfEdgesByDestination.end()){
                halfEdgesByDestination.insert({id, vector<shared_ptr<HalfEdge>>()});
            }
            halfEdgesByDestination[id].push_back(previous);
            face->halfEdge = firstHalfEdge;
            faces.push_back(face);
        }
    };
    for(int i = 0; i < object.faces.size(); ++i){
        makeFaceTri(object.faces[i]);
    }
    for(const shared_ptr<HalfEdge> &halfEdge : halfEdges){
        shared_ptr<HalfEdge> twin = halfEdge->twin.lock();
        if(!twin){
            int origin = halfEdge->vertex->id;
            int destination = halfEdge->next->vertex->id;
            for(const shared_ptr<HalfEdge> &twin : halfEdgesByDestination[origin]){
                if(twin->vertex->id == destination){
                    if(!twin->twin.lock()){
                        halfEdge->twin = twin;
                        twin->twin = halfEdge;
                        break;
                    }
                }
            }
        }
    }
    for(int i = 0; i < vertices.size(); ++i){
        if(vertices[i] == nullptr){
            vertices[i] = make_shared<Vertex>(i);
            vertices[i]->position = indexVertex[i].position;
            vertices[i]->colour = indexVertex[i].colour;
            vertices[i]->textureCoordinates = indexVertex[i].textureCoords;
        }
    }
    object.vertices = vertices;
    object.faces = faces;
    object.halfEdges = halfEdges;
}

/*by Daniel Jolley-Rogers u7511912
runs all of the mesh checking functions on an object*/
void checkMesh(Object &object, const string &fileName){
    cout << "checking characteristics of " << fileName << "\n";
    cout << "\tchecking if object tri";
    if(!objectTri(object)){
        cout << " - not tri\n";
        if(objectQuad(object)){
            cout << "\t\tobject quad\n";
        }
        cout << "\t\tattempting to make object triangle mesh";
        makeObjectTri(object);
        if(objectTri(object)){
            cout << " - success\n";
        }
        else{
            cout << " - failed\n";
        }
    }
    else{
        cout << " - tri\n";
    }
    cout << "\tchecking consistency";
    if(!objectFacesConsistent(object)){
        cout << " - not consistent\n";
        cout << "\t\tattempting to make object consistently faced";
        makeObjectFacesConsistent(object);
        if(objectFacesConsistent(object)){
            cout << " - success\n";
        }
        else{
            cout << " - failed\n";
        }
    }
    else{
        cout << " - consistent\n";
    }
    cout << "\tchecking connected";
    if(!objectConnected(object)){
        cout << " - not connected\n";
        cout << "\t\tattempting to make object connected";
        makeObjectConnected(object);
        if(objectConnected(object)){
            cout << " - success\n";
        }
        else{
            cout << " - failed\n";
        }
    }
    else{
        cout << " - connected\n";
    }
    cout << "\tchecking manifold";
    if(!objectManifold(object)){
        cout << " - not manifold\n";
        cout << "\t\tmaking object pseudo manifold\n";
        makeObjectManifold(object);
    }
    else{
        cout << " - manifold\n";
    }
    cout << "\tchecking closed";
    if(!objectClosed(object)){
        cout << " - not closed\n";
        cout << "\t\tattempting to make object closed";
        makeObjectClosed(object);
        if(objectClosed(object)){
            cout << " - success\n";
        }
        else{
            cout << " - failed\n";
        }
    }
    else{
        cout << " - closed\n";
    }
}

/*by Daniel Jolley-Rogers u7511912
helper struct to compare Eigen vectors for a map*/
struct LexographicLess {
    template<class T>
    bool operator()(T const& lhs, T const& rhs) const {
      return lexicographical_compare(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
    }
};

/*by Daniel Jolley-Rogers u7511912
saves an object as a .obj file*/
bool saveMeshASOBJ(const Object &object, string fileNameQualifier){
    int positionCounter = 1;
    int textureCounter = 1;
    int normalCounter = 1;
    map<int, Vector3f> indexToPosition;
    map<int, Vector2f> indexToTexture;
    map<int, Vector3f> indexToNormal;
    map<Vector3f, int, LexographicLess> positionToIndex;
    map<Vector2f, int, LexographicLess> textureToIndex;
    map<Vector3f, int, LexographicLess> normalToIndex;

    for(const shared_ptr<Vertex> &vertex : object.vertices){
        if(positionToIndex.find(vertex->position) == positionToIndex.end()){
            positionToIndex[vertex->position] = positionCounter;
            indexToPosition[positionCounter] = vertex->position;
            positionCounter += 1;
        }
        if(textureToIndex.find(vertex->textureCoordinates) == textureToIndex.end()){
            textureToIndex[vertex->textureCoordinates] = textureCounter;
            indexToTexture[textureCounter] = vertex->textureCoordinates;
            textureCounter += 1;
        }
        if(normalToIndex.find(vertex->normal) == normalToIndex.end()){
            normalToIndex[vertex->normal] = normalCounter;
            indexToNormal[normalCounter] = vertex->normal;
            normalCounter += 1;
        }
    }
    //make new file name
    filesystem::path p(object.objFile);
    filesystem::path filePath = p.parent_path() / (fileNameQualifier + p.filename().string());
    ofstream outFile(filePath);
    if(!outFile.is_open()){
        cerr << "Failed to open file: " << filePath << "\n";
        return false;
    }
    outFile << "mtllib " << object.material.mtlFile << "\n";
    outFile << "\n\n";

    int precision = 4;
    outFile << fixed << setprecision(precision);

    //write the vertices
    int numVertices = 0;
    for(auto indexPosition : indexToPosition){
        numVertices += 1;
        Vector3f position = indexPosition.second;
        outFile << "v " << position.x() << " " << position.y() << " " << position.z() << "\n";
    }
    for(auto indexNormal : indexToNormal){
        Vector3f normal = indexNormal.second;
        outFile << "vn " << normal.x() << " " << normal.y() << " " << normal.z() << "\n";
    }
    for(auto indexTexture : indexToTexture){
        Vector2f texture = indexTexture.second;
        outFile << "vt " << texture.x() << " " << texture.y() << " 0.0\n";
    }
    outFile << "\n# num vertices: " << numVertices << "\n";
    //add some space
    outFile << "\n\n";
    //then the mtl file
    outFile << "usemtl " << filesystem::path(object.material.mtlFile).stem().string() << "\n";
    //then some more space
    outFile << "\n\n";

    //then add the faces
    for(const shared_ptr<Face> &face : object.faces){
        outFile << "f";
        shared_ptr<HalfEdge> startHalfEdge = face->halfEdge.lock();
        if(!startHalfEdge){
            cerr << "Warning: face with expired halfEdge, skipping\n";
            continue;
        }
        shared_ptr<HalfEdge> current = startHalfEdge;
        do{
            shared_ptr<Vertex> vertex = current->vertex;
            int posIndex = positionToIndex[vertex->position];
            int texIndex = textureToIndex[vertex->textureCoordinates];
            int normIndex = normalToIndex[vertex->normal];
            outFile << " " << posIndex << "/" << texIndex << "/" << normIndex;
            current = current->next;
        }while(current != startHalfEdge);
        outFile << "\n";
    }
    outFile << "\n# num faces: " << object.faces.size();
    return true;
}

/*by Daniel Jolley-Rogers u7511912
loads a mesh, runs the checkers on it and then saves the mesh*/
int main(int argc, char** argv){
    string fileString;
    if(argc >= 2){
        fileString = argv[1];
    }
    filesystem::path filePath = fileString;
    if(filePath.extension() != ".obj"){
        throw runtime_error("not an obj file");
    }
    Mesh mesh;
    Object object;
    try{
        cout << "running\n";
        cout << "\tloading " << filePath << "\n";
        mesh = loadFile(filePath);
        cout << "converting mesh to halfEdge\n";
        object = meshToHalfEdge(mesh);
        object.objFile = filePath;
        cout << "saving mesh for testing\n";
        saveMeshASOBJ(object, "beforeCheck_");
        checkMesh(object, filePath);
        cout << "saving new obj\n";
        saveMeshASOBJ(object, "afterCheck_");
        cout << "done\n";
    }
    catch(string message){
        cerr << "exception message: " << message << "\n";
    }
}
