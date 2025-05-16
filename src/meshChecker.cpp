#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <set>
#include <stack>
#include <map>
#include "Eigen/Dense"

#include <meshChecker.hpp>
#include <object.hpp>

using namespace std;
using namespace Eigen;

bool objectClosed(const Object &object){
    for(shared_ptr<HalfEdge> halfEdge : object.halfEdges){
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

void makeObjectClosed(Object &object){
    //TODO
}

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
        for (std::shared_ptr<Vertex> neighbour : vertex->getNeighbourVertices()) {
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

void makeObjectConnected(Object &object){
    //TODO
}

bool objectManifold(const Object &object){
    //to check edge manifoldness, make sure their are exactly 2 faces on each edge.
    //first check to make sure only 2 faces share an edge
    vector<vector<int>> numEdgesBetweenVertices(object.vertices.size(), vector<int>(object.vertices.size(), 0));
    for(shared_ptr<Face> face : object.faces){
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
    for(shared_ptr<HalfEdge> halfEdge : object.halfEdges){
        if(!halfEdge->face){
            return false;
        }
        int id = halfEdge->vertex->id;
        startingPoints[id].push_back(halfEdge->id);
    }
    //check vertex manifoldness by exploring vertex fans
    int index = 0;
    int maxFan = object.vertices.size();

    for(shared_ptr<Vertex> vertex : object.vertices){
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

void makeObjectManifold(Object &object){
    //TODO
}

bool objectFacesConsistent(const Object &object){
    //two adjacent polygons have a consistant orientation if touching edges face opposite directions
    //make a bool for each origin vertex - destination vertex combo and set it to false
    vector<vector<bool>> originToDest(object.vertices.size(), vector<bool>(object.vertices.size(), false));
    for(shared_ptr<HalfEdge> halfEdge : object.halfEdges){
        int origin = halfEdge->vertex->id;
        int destination = halfEdge->next->vertex->id;
        //if the current half edge's origin and destination have been encountered return false
        if(originToDest[origin][destination]){
            return false;
        }
        //set the current origin and destination combo to true
        originToDest[origin][destination] = true;
    }
    return true;
}

void makeObjectFacesConsistent(Object &object){
    vector<shared_ptr<Face>> consistentFaces;
    vector<shared_ptr<HalfEdge>> consistentHalfEdges;
    set<int> visited;
    int numVertices = object.vertices.size();
    //get a matrix of what faces connect to what edges
    vector<vector<vector<shared_ptr<Face>>>> edges(numVertices, vector<vector<shared_ptr<Face>>>(numVertices));
    vector<shared_ptr<Face>> faces = object.faces;
    for(shared_ptr<Face> face : object.faces){
        shared_ptr<HalfEdge> halfEdge = face->halfEdge.lock();
        shared_ptr<HalfEdge> previous;
        int startID = halfEdge->id;
        do{
            previous = halfEdge;
            halfEdge = halfEdge->next;
            int v1 = previous->vertex->id;
            int v2 = halfEdge->vertex->id;
            edges[v1][v2].push_back(face);
            edges[v2][v1].push_back(face);
        }
        while(halfEdge->id != startID);
    }

    //recursion lambda
    auto exploreFaces = [&](const auto &self, shared_ptr<Face> face, bool reverse) -> void {
        if(visited.find(face->id) != visited.end()){
            //if face needs to be reversed but it has already been visited that is a contradiction
            if(reverse){
                throw runtime_error(string("can't make faces consistently faced"));
            }
            //otherwise have reached end of this branch
            return;
        }
        //directions - [from, to]
        vector<vector<int>> directions; 

        vector<shared_ptr<HalfEdge>> clones;
        shared_ptr<HalfEdge> current = face->halfEdge.lock();
        int startID = current->id;
        do {
            HalfEdge newHalfEdge = *current;
            shared_ptr<HalfEdge> newHalfEdgePointer = make_shared<HalfEdge>(newHalfEdge);
            clones.push_back(newHalfEdgePointer);
            consistentHalfEdges.push_back(newHalfEdgePointer);
            current = current->next;
        } 
        while (current->id != startID);

        int n = clones.size();
        for(int i = 0; i < n; ++i){
            clones[i]->twin.lock() = nullptr;
            if(reverse){
                clones[i]->next = clones[(i - 1 + n) % n];
                clones[i]->previous = clones[(i + 1 + n) % n];
            }
            else{
                clones[i]->next = clones[(i + 1) % n];
                clones[i]->previous = clones[(i - 1 + n) % n];
            }
            directions.push_back({clones[i]->vertex->id, clones[i]->next->vertex->id});
        }
        face->halfEdge = clones[0];
        visited.insert(face->id);
        consistentFaces.push_back(face);
        
        for(std::vector<int> directedEdge : directions){
            int from = directedEdge[0];
            int to = directedEdge[1];
            vector<shared_ptr<Face>> adjacent = edges[from][to];
            for(shared_ptr<Face> adj : adjacent){
                if(adj->id == face -> id){
                    continue;
                }
                current = adj->halfEdge.lock();
                startID = current->id;
                do{
                    shared_ptr<HalfEdge> previous = current;
                    current = current->next;
                    int adjFrom = previous->vertex->id;
                    int adjTo = current->vertex->id;
                    if(from == adjFrom && to == adjTo){
                        self(self, adj, true);
                        break;
                    }
                    else if(from == adjTo && to == adjFrom){
                        self(self, adj, false);
                        break;
                    }
                }
                while(current->id != startID);
            }
        }
        return;
    };
    //handle disconnected faces by starting in multiple spots
    //(faces will only be consistent with faces they are connected to)
    for(shared_ptr<Face> face : faces){
        if(visited.count(face->id) == 0){
            exploreFaces(exploreFaces, face, false);
        }
    }

    for(int i = 0; i < edges.size(); ++i){
        for(int j = i + 1; j < edges.size(); ++j){
            vector<shared_ptr<Face>> currentEdgeFaces = edges[i][j];
            //assume there are only at most 2 entries in currentEdgeFaces
            vector<shared_ptr<HalfEdge>> adjacentHalfEdges;
            int numIterations = 0;
            for(shared_ptr<Face> face : currentEdgeFaces){
                numIterations += 1;
                if(numIterations > 2){
                    break;
                }
                shared_ptr<HalfEdge> current = face->halfEdge.lock();
                int startID = current->id;
                do{
                    shared_ptr<HalfEdge> previous = current;
                    current = current->next;
                    int from = previous->vertex->id;
                    int to = current->vertex->id;
                    if((i == from && j == to) || (j == to && i == from)){
                        adjacentHalfEdges.push_back(previous);
                        break;
                    }
                }
                while(current->id != startID);

            }
            if(adjacentHalfEdges.size() == 2){
                adjacentHalfEdges[0]->twin = adjacentHalfEdges[1];
                adjacentHalfEdges[1]->twin = adjacentHalfEdges[0];
            }
        }
    }
    object.faces = consistentFaces;
    object.halfEdges = consistentHalfEdges;
}

bool objectTri(const Object &object){
    for(int i = 0; i < object.faces.size(); ++i){
        if(object.faces[i]->getHalfEdges().size() > 3){
            return false;
        }
    }
    return true;
}

namespace seidel{
    struct Point{
        Vector3f position;
        Vector3f colour;
        Vector2f textureCoords;
        Point(const shared_ptr<Vertex> &vert){
            position = vert->position;
            colour = vert->colour;
            textureCoords = vert->textureCoordinates;
        }
        Point(Vector3f _position, Vector3f _colour, Vector2f _textureCoords): position(_position), colour(_colour), textureCoords(_textureCoords){}
        Point(){}
    };

    vector<vector<Point>> triangulateQuad(Point leftOrigin, Point leftDest, Point rightOrigin, Point rightDest){
        vector<vector<Point>> triangles;
        //maintain orientation
        //if leftDest higher then left origin then the orientation is clockwise
        if(leftDest.position.y() > leftOrigin.position.y()){
            //check if already triangle
            if(leftDest.position == rightOrigin.position){
                Point A = leftOrigin;
                Point B = leftDest;
                Point C = rightDest;
                triangles.push_back({A, B, C});
                return triangles;
            }
            else if(rightDest.position == leftOrigin.position){
                Point A = leftOrigin;
                Point B = leftDest;
                Point C = rightOrigin;
                triangles.push_back({A, B, C});
                return triangles;
            }
            //form triangles from vertices
            Point A1 = leftOrigin;
            Point B1 = leftDest;
            Point C1 = rightDest;
            triangles.push_back({A1, B1, C1});
    
            Point A2 = leftDest;
            Point B2 = rightOrigin;
            Point C2 = rightDest;
            triangles.push_back({A2, B2, C2});
        }
        //else anticlockwise
        else{
            if(leftOrigin.position == rightDest.position){
                Point A = leftOrigin;
                Point B = leftDest;
                Point C = rightOrigin;
                triangles.push_back({A, B, C});
                return triangles;
            }
            else if(rightOrigin.position == leftDest.position){
                Point A = leftOrigin;
                Point B = leftDest;
                Point C = rightDest;
                triangles.push_back({A, B, C});
                return triangles;
            }
            Point A1 = leftOrigin;
            Point B1 = leftDest;
            Point C1 = rightDest;
            triangles.push_back({A1, B1, C1});
    
            Point A2 = leftDest;
            Point B2 = rightOrigin;
            Point C2 = rightDest;
            triangles.push_back({A2, B2, C2});
        }
        return triangles;
    }    

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
    
    shared_ptr<Node> processVertexNode(const shared_ptr<Vertex> &vert, shared_ptr<Node> &tree, vector<shared_ptr<HalfEdge>> faceEdges, int &nodeID, int &trapID){
        if(tree == nullptr){
            //nodeType only root when tree is empty
            shared_ptr<SeidelRay> ray = make_shared<SeidelRay>(vert, faceEdges);
            tree = make_shared<Node>(nodeID++);
            tree->nodeType = Node::VERTEX;
            tree->vertex = vert;
    
            tree->left = make_shared<Node>(Node(nodeID++));
            tree->left->parent = tree;
            std::shared_ptr<Trapezoid> bottomTrap = make_shared<Trapezoid>(tree->left, nullptr, ray, trapID++);
            tree->left->nodeType = Node::TRAPAZOID;
            tree->left->trapezoid = bottomTrap;
    
            tree->right = make_shared<Node>(nodeID++);
            tree->right->parent = tree;
            std::shared_ptr<Trapezoid> topTrap = make_shared<Trapezoid>(tree->right, ray, nullptr, trapID++);
            tree->right->nodeType = Node::TRAPAZOID;
            tree->right->trapezoid = topTrap;
            topTrap->down.push_back(bottomTrap);
            bottomTrap->up.push_back(topTrap);
            return tree;
        }
        shared_ptr<Node> currentNode = tree;
        while(true){
            if(currentNode->nodeType == Node::VERTEX){
                Vector3f a = vert->position;
                Vector3f b = currentNode->vertex.lock()->position;
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
                shared_ptr<HalfEdge> e = currentNode->edge.lock();
                Vector3f origin = e->vertex->position;
                Vector3f dest = e->next->vertex->position;
                float yValue = vert->position.y();
                Vector3f unitVector = (dest - origin).normalized();
                float mult = (yValue - origin.y()) / unitVector.y();
                Vector3f intercept = origin + (unitVector * mult);
                Vector3f a = vert->position;
            }
            else{
                //if not vertex or edge must be trapezoid
                shared_ptr<Trapezoid> old = currentNode->trapezoid;
                shared_ptr<SeidelRay> ray = make_shared<SeidelRay>(vert, faceEdges);
                shared_ptr<SeidelRay> oldHighRay = old->highRay;
                shared_ptr<SeidelRay> oldLowRay = old->lowRay;
                currentNode->nodeType = Node::VERTEX;
                currentNode->trapezoid = nullptr;
                currentNode->vertex = vert;
                currentNode->left = make_shared<Node>(nodeID++);
                currentNode->left->parent = currentNode;
                std::shared_ptr<Trapezoid> bottomTrap = make_shared<Trapezoid>(currentNode->left, oldLowRay, ray, trapID++);
                currentNode->left->nodeType = Node::TRAPAZOID;
                currentNode->left->trapezoid = bottomTrap;
                currentNode->right = make_shared<Node>(nodeID++);
                currentNode->right->parent = currentNode;
                std::shared_ptr<Trapezoid> topTrap = make_shared<Trapezoid>(currentNode->right, ray, oldHighRay, trapID++);
                currentNode->right->nodeType = Node::TRAPAZOID;
                currentNode->right->trapezoid = topTrap;
                topTrap->up = old->up;
                bottomTrap->down = old->down;
                topTrap->down.push_back(bottomTrap);
                bottomTrap->up.push_back(topTrap);
                for(shared_ptr<Trapezoid> above : topTrap->up){
                    replace(above->down.begin(), above->down.end(), old, topTrap);
                }
                for(shared_ptr<Trapezoid> below : bottomTrap->down){
                    replace(below->up.begin(), below->up.end(), old, bottomTrap);
                }
                return currentNode;
            }
        }
    }
    
    void insertEdge(const shared_ptr<Node> &node, const shared_ptr<HalfEdge> &halfEdge, int &nodeID, int &trapID){
        shared_ptr<Trapezoid> old = node->trapezoid;
        shared_ptr<SeidelRay> highRay = old->highRay;
        shared_ptr<SeidelRay> lowRay = old->lowRay;
        node->nodeType = Node::EDGE;
        node->trapezoid = nullptr;
        node->edge = halfEdge;
        node->left = make_shared<Node>(nodeID++);
        node->left->parent = node;
        std::shared_ptr<Trapezoid> bottomTrap = make_shared<Trapezoid>(node->left, lowRay, highRay, trapID++);
        node->left->nodeType = Node::TRAPAZOID;
        node->left->trapezoid = bottomTrap;
        node->right = make_shared<Node>(Node(nodeID++));
        node->right->parent = node;
        std::shared_ptr<Trapezoid> topTrap = make_shared<Trapezoid>(node->right, lowRay, highRay, trapID++);
        node->right->nodeType = Node::TRAPAZOID;
        node->right->trapezoid = topTrap;
        Vector3f a;
        Vector3f b;
        vector<shared_ptr<Node>> children = {node->left, node->right};
        //get new ups and downs, and set neighbours
        for(shared_ptr<Trapezoid> above : old->up){
            auto iter = find(above->down.begin(), above->down.end(), old);
            if ( iter != above->down.end() ) {
                above->down.erase(iter);
            }
            //get lower left above x coord
            float lowerLeftAbove;
            shared_ptr<HalfEdge> lseg = above->lseg.lock();
            if(!lseg){
                lowerLeftAbove = -FLT_MAX;
            }
            else{
                a = lseg->vertex->position;
                b = lseg->next->vertex->position;
                if(a.y() < b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                    lowerLeftAbove = a.x();
                }
                else{
                    lowerLeftAbove = b.x();
                }
            }
    
            //get lower right above x coord
            float lowerRightAbove;
            shared_ptr<HalfEdge> rseg = above->rseg.lock();
            if(!rseg){
                lowerRightAbove = FLT_MAX;
            }
            else{
                a = rseg->vertex->position;
                b = rseg->next->vertex->position;
                if(a.y() < b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                    lowerRightAbove = a.x();
                }
                else{
                    lowerRightAbove = b.x();
                }
            }
            for(shared_ptr<Node> child : children){
                //get upper left child x coord
                float upperLeftChild;
                lseg = child->trapezoid->lseg.lock();
                if(!lseg){
                    upperLeftChild = -FLT_MAX;
                }
                else{
                    a = lseg->vertex->position;
                    b = lseg->next->vertex->position;
                    if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                        upperLeftChild = a.x();
                    }
                    else{
                        upperLeftChild = b.x();
                    }
                }
                //get upper right child x coord
                float upperRightChild;
                rseg = child->trapezoid->rseg.lock();
                if(!rseg){
                    upperRightChild = FLT_MAX;
                }
                else{
                    a = rseg->vertex->position;
                    b = rseg->next->vertex->position;
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
        for(shared_ptr<Trapezoid> below : old->down){
            auto iter = find(below->up.begin(), below->up.end(), old);
            if ( iter != below->up.end() ) {
                below->up.erase(iter);
            }
            //get upper left below x coord
            float upperLeftBelow;
            shared_ptr<HalfEdge> lseg = below->lseg.lock();
            if(!lseg){
                upperLeftBelow = -FLT_MAX;
            }
            else{
                a = lseg->vertex->position;
                b = lseg->next->vertex->position;
                if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                    upperLeftBelow = a.x();
                }
                else{
                    upperLeftBelow = b.x();
                }
            }
            //get upper right below x coord
            float upperRightBelow;
            shared_ptr<HalfEdge> rseg = below->rseg.lock();
            if(!rseg){
                upperRightBelow = FLT_MAX;
            }
            else{
                a = rseg->vertex->position;
                b = rseg->next->vertex->position;
                if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                    upperRightBelow = a.x();
                }
                else{
                    upperRightBelow = b.x();
                }
            }
            for(shared_ptr<Node> child : children){
                //get lower left child x coord
                float lowerLeftChild;
                lseg = child->trapezoid->lseg.lock();
                if(!lseg){
                    lowerLeftChild = -FLT_MAX;
                }
                else{
                    a = lseg->vertex->position;
                    b = lseg->next->vertex->position;
                    if(a.y() < b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                        lowerLeftChild = a.x();
                    }
                    else{
                        lowerLeftChild = b.x();
                    }
                }
                //get lower right child x coord
                float lowerRightChild;
                rseg = child->trapezoid->rseg.lock();
                if(!rseg){
                    lowerRightChild = FLT_MAX;
                }
                else{
                    a = rseg->vertex->position;
                    b = rseg->next->vertex->position;
                    if(a.y() < b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
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
    
    void processEdgeNode(const shared_ptr<HalfEdge> &halfEdge, const shared_ptr<Node> &lower, const shared_ptr<Node> &upper, int &nodeID, int &trapID){
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
                Vector3f o = halfEdge->vertex->position;
                Vector3f n = halfEdge->next->vertex->position;
                if(o.y() > n.y() || (o.y() == n.y() && tieBreak(o, n) == o)){
                    targetTop = halfEdge->vertex;
                }
                else{
                    targetTop = halfEdge->next->vertex;
                }
                shared_ptr<Vertex> currentTop;
                shared_ptr<HalfEdge> edge = currentNode->edge.lock();
                o = edge->vertex->position;
                n = edge->next->vertex->position;
                if(o.y() > n.y() || (o.y() == n.y() && tieBreak(o, n) == o)){
                    currentTop = edge->vertex;
                }
                else{
                    currentTop = edge->next->vertex;
                }
                Vector3f a = targetTop->position;
                Vector3f b = currentTop->position;
                if(a.x() > b.x() || (a.x() == b.x() && tieBreak(a, b) == a)){
                    currentNode = currentNode->right;
                }
                else{
                    currentNode = currentNode->left;
                }
            }
            else{
                placeInserted = currentNode->id;
                insertEdge(currentNode, halfEdge, nodeID, trapID);
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
                Vector3f o = halfEdge->vertex->position;
                Vector3f n = halfEdge->next->vertex->position;
                if(o.y() < n.y() || (o.y() == n.y() && tieBreak(o, n) == o)){
                    targetBottom = halfEdge->vertex;
                }
                else{
                    targetBottom = halfEdge->next->vertex;
                }
                shared_ptr<Vertex> currentBottom;
                shared_ptr<HalfEdge> edge = currentNode->edge.lock();
                o = edge->vertex->position;
                n = edge->next->vertex->position;
                if(o.y() < n.y() || (o.y() == n.y() && tieBreak(o, n) == o)){
                    currentBottom = edge->vertex;
                }
                else{
                    currentBottom = edge->next->vertex;
                }
                Vector3f a = targetBottom->position;
                Vector3f b = currentBottom->position;
                if(a.x() > b.x() || (a.x() == b.x() && tieBreak(a, b) == a)){
                    currentNode = currentNode->right;
                }
                else{
                    currentNode = currentNode->left;
                }
            }
            else{
                insertEdge(currentNode, halfEdge, nodeID, trapID);
                break;
            }
        }
    }
    
    inline Point yInterceptFinder(const shared_ptr<Vertex> origin, const shared_ptr<Vertex> dest, float yIntercept){
        Vector3f edgeVector = dest->position - origin->position;
        float mult = (yIntercept - origin->position.y()) / (edgeVector.y());
        Vector3f position = origin->position + (mult * (edgeVector));
        Vector3f colour = (origin->colour * (1 - mult)) + (dest->colour * mult);
        Vector2f textureCoords = (origin->textureCoordinates * (1 - mult)) + (dest->textureCoordinates * mult);
        return Point(position, colour, textureCoords);
    }
    
    inline Vector3f xInterceptFinder(Vector3f origin, Vector3f dest, float xIntercept){
        float mult = (xIntercept - origin.x()) / (dest.x() - origin.x());
        return origin + (mult * (dest - origin));
    }
    
    

    //Seidel's algorithm for decomposing to trapezoids (quadralaterials)
    vector<vector<Point>> trapodize(const shared_ptr<Face> &face){
        vector<vector<Point>> triangles;
        int trapID = 0;
        int nodeID = 0;
        //I use SEIDEL'S algorithm to perform trapezoidation
        //source: http://www.polygontriangulation.com/2018/07/triangulation-algorithm.html
        vector<shared_ptr<HalfEdge>> faceEdges = face->getHalfEdges();
        shared_ptr<Node> tree;
        //get arbitrary edge from list of edges
        for(shared_ptr<HalfEdge> edge : faceEdges){
            //get edge vertices
            shared_ptr<Vertex> v1 = edge->vertex;
            shared_ptr<Vertex> v2 = edge->next->vertex;
            //find higher and lower vertex
            Vector3f a = v1->position;
            Vector3f b = v2->position;
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
            shared_ptr<Node> upperNode = processVertexNode(higher, tree, faceEdges, nodeID, trapID);
            shared_ptr<Node> lowerNode = processVertexNode(lower, tree, faceEdges, nodeID, trapID);
            //place edge below upper node but above lower node
            processEdgeNode(edge, lowerNode, upperNode, nodeID, trapID);
        }
        //get list of trapezoids and make sure their edges are finalised
        //and set their validatiy
        vector<shared_ptr<Trapezoid>> trapezoids;
        stack<shared_ptr<Node>> nodeStack;
        if(!tree){
            cerr << "seidel root node not initilised\n";
        }
        nodeStack.push(tree);
        while(!nodeStack.empty()) {
            shared_ptr<Node> node = nodeStack.top(); 
            nodeStack.pop();
            if(node->nodeType == Node::TRAPAZOID){
                node->trapezoid->setLeftEdge(node);
                node->trapezoid->setRightEdge(node);
                node->trapezoid->inPolygon();
                trapezoids.push_back(node->trapezoid);
            }
            else{
                nodeStack.push(node->left);
                nodeStack.push(node->right);
            }
        }
        //transform trapezoids into triangles
        for(shared_ptr<Trapezoid> trap : trapezoids){
            if(!trap->validState){
                continue;
            }
            //get vertices of trapezoid
            //use rays and side segments to interporlate the four vertices
            float highY = trap->highRay->yValue;
            float lowY = trap->lowRay->yValue;
            shared_ptr<HalfEdge> lseg = trap->lseg.lock();
            shared_ptr<HalfEdge> rseg = trap->rseg.lock();
            shared_ptr<Vertex> leftEdgeOrigin = lseg->vertex;
            shared_ptr<Vertex> leftEdgeDest = lseg->next->vertex;
            shared_ptr<Vertex> rightEdgeOrigin = rseg->vertex;
            shared_ptr<Vertex> rightEdgeDest = rseg->next->vertex;
    
            Point leftLowVertex = yInterceptFinder(leftEdgeOrigin, leftEdgeDest, lowY);
            Point leftHighVertex = yInterceptFinder(leftEdgeOrigin, leftEdgeDest, highY);
            Point rightLowVertex = yInterceptFinder(rightEdgeOrigin, rightEdgeDest, lowY);
            Point rightHighVertex = yInterceptFinder(rightEdgeOrigin, rightEdgeDest, highY);
    
            Point leftOrigin;
            Point leftDest;
            Point rightOrigin;
            Point rightDest;
            if(leftEdgeOrigin->position.y() > leftEdgeDest->position.y() || (leftEdgeOrigin->position.y() == leftEdgeDest->position.y() && tieBreak(leftEdgeOrigin->position, leftEdgeDest->position) == leftEdgeOrigin->position)){
                leftOrigin = leftHighVertex;
                leftDest = leftLowVertex;
            }
            else{
                leftOrigin = leftLowVertex;
                leftDest = leftHighVertex;
            }
            if(rightEdgeOrigin->position.y() > rightEdgeDest->position.y() || (rightEdgeOrigin->position.y() == rightEdgeDest->position.y() && tieBreak(rightEdgeOrigin->position, rightEdgeDest->position) == rightEdgeOrigin->position)){
                rightOrigin = rightHighVertex;
                rightDest = rightLowVertex;
            }
            else{
                rightOrigin = rightLowVertex;
                rightDest = rightHighVertex;
            }
            vector<vector<Point>> currentTriangles = triangulateQuad(leftOrigin, leftDest, rightOrigin, rightDest);
            triangles.insert(triangles.end(), currentTriangles.begin(), currentTriangles.end());
        }
        return triangles;
    }
    
    //checks if a given face has a self intersection and returns a list of the points they occur
    //and the edges they occur on
    vector<pair<Vector3f, vector<shared_ptr<HalfEdge>>>> selfIntersectChecker(const shared_ptr<Face> &face){
        //using alogrithm found here:
        //https://www.wyzant.com/resources/answers/696697/check-if-polygon-is-self-intersecting
        //get pairs of nonAdjacent edges
        vector<shared_ptr<HalfEdge>> halfEdges = face->getHalfEdges();
        set<vector<shared_ptr<HalfEdge>>> edgePairs;
        vector<pair<Vector3f, vector<shared_ptr<HalfEdge>>>> selfInteceptPoints;
        //intersection can only occur with 4 or more vertices
        if(face->getVertices().size() < 4){
            return selfInteceptPoints;
        }
        //get all edge pairs which aren't neighbours
        for(int i = 0; i < halfEdges.size(); ++i){
            for(int j = i + 1; j < halfEdges.size(); ++j){
                //check to make sure halfedges are not neighbours
                if(halfEdges[i]->next->id == halfEdges[j]->id || halfEdges[j]->next->id == halfEdges[i]->id){
                    continue;
                }
                //check to make sure halfEdges are not in set in other order
                if(edgePairs.count({halfEdges[j], halfEdges[i]}) != 0){
                    continue;
                }
                edgePairs.insert({halfEdges[i], halfEdges[j]});
            }
        }
        //check if edge pairs intersect eachother
        for(vector<shared_ptr<HalfEdge>> edgePair : edgePairs){
            shared_ptr<HalfEdge> he1 = edgePair[0];
            shared_ptr<HalfEdge> he2 = edgePair[1];
            vector<Vector3f> s1 = {he1->vertex->position, he1->next->vertex->position};
            vector<Vector3f> s2 = {he2->vertex->position, he2->next->vertex->position};
            float s1XMin = min({s1[0].x(), s1[1].x()});
            float s1XMax = max({s1[0].x(), s1[1].x()});
            float s2XMin = min({s2[0].x(), s2[1].x()});
            float s2XMax = max({s2[0].x(), s2[1].x()});
            float lowerMax = max({s1XMin, s2XMin});
            float upperMin = min({s1XMax, s2XMax});
            //check if overlap possible
            if(lowerMax < upperMin){
                float s1dx = s1[1].x() - s1[0].x();
                float s1dy = s1[1].y() - s1[0].y();
                float s2dx = s2[1].x() - s2[0].x();
                float s2dy = s2[1].y() - s2[0].y();
                float m1 = s1dy / s2dx;
                float m2 = s2dy / s2dx;
                float x;
                //first find at what x coord a possible intersection would have to be
                if(s1dx == 0){
                    //if one of the sides is straight up and down the x intersection is given by that side
                    x = s1[0].x();
                }
                else if(s2dx == 0){
                    //if one of the sides is straight up and down the x intersection is given by that side
                    x = s2[0].x();
                }
                else if(m1 == m2){
                    //if the lines are parrallel the x can be any x between lowerMax and upperMin
                    float difference = upperMin - lowerMax;
                        x = lowerMax + (0.5 * difference);
                }
                else{
                    //otherwise we know the lines are not parrellel so we just need to solve for x
                    //using equations:
                    // y = (m1 * x) + b1
                    // y = (m2 * x) + b2
                    //where m = rise / run and b can be found by subsituting 1 end of each side
                    //we can solve for x using subsitution
                    m1 = (s1dy / s2dx);
                    m2 = (s2dy / s2dx);
                    float b1 = s1[0].y() - (m1 * s1[0].x());
                    float b2 = s2[0].y() - (m2 * s2[1].x());
                    x = (b1 - b2) / (m1 - m2);
                }
                Vector3f s1Intercept = xInterceptFinder(s1[0], s1[1], x);
                Vector3f s2Intercept = xInterceptFinder(s2[0], s2[1], x);
                //just need to check to see if intercepts are the same to see if there is a self intersection here
                if(s1Intercept == s2Intercept){
                    selfInteceptPoints.push_back({s1Intercept, {he1, he2}});
                }
            }
        }
        return selfInteceptPoints;
    }

    vector<vector<Point>> triangulate(const shared_ptr<Face> &face){
        vector<vector<Point>> triangles;
        //check how many sides current face has
        shared_ptr<HalfEdge> HalfEdge = face->halfEdge.lock();
        auto current = HalfEdge;
        int startID = current->id;
        int numEdges = 0;
        int selfIntersectAmount = selfIntersectChecker(face).size();
        do{
            current = current->next;
            numEdges += 1;
        }
        while(current->id != startID);
        //if it is already a triangle, return unchanged
        if(numEdges == 3){
            Point a = Point(HalfEdge->vertex);
            Point b = Point(HalfEdge->next->vertex);
            Point c = Point(HalfEdge->next->next->vertex);
            triangles.push_back({a, b, c});
        }
        //if it is a quad and has no self intersections use basic quad triangulation
        else if (numEdges == 4){// && selfIntersectAmount == 0){
            //for left and right edges all that matters is that they are opposite edges actual orientation doesn't matter
            Point leftOrigin = Point(HalfEdge->vertex);
            Point leftDest = Point(HalfEdge->next->vertex);
            Point rightOrigin = Point(HalfEdge->next->next->vertex);
            Point rightDest = Point(HalfEdge->next->next->next->vertex);
            triangles = triangulateQuad(leftOrigin, leftDest, rightOrigin, rightDest);
        }
        //else use Seidel's
        else{
            triangles = trapodize(face);
        }
        return triangles;
    }
}

void makeObjectTri(Object &object){
    vector<shared_ptr<Vertex>> vertices;
    vector<shared_ptr<Face>> faces;
    vector<shared_ptr<HalfEdge>> halfEdges;
    map<vector<float>, int> vertexIndex;
    map<int, seidel::Point> indexVertex;
    int vertexID, faceID, halfEdgeID;
    vertexID = faceID = halfEdgeID = 0;
    map<int, vector<shared_ptr<HalfEdge>>> halfEdgesByDestination;

    auto makeFaceTri = [&](const shared_ptr<Face> face) -> void{
        vector<vector<seidel::Point>> faceTriangles = seidel::triangulate(face);
        for(vector<seidel::Point> triangle : faceTriangles){
            vector<int> faceCoords = {};
            for(seidel::Point point : triangle){
                Vector3f position = point.position;
                vector<float> vectorPoint = {position.x(), position.y(), position.z()};
                if(vertexIndex.count(vectorPoint) == 0){
                    vertexIndex.insert({vectorPoint, vertexID});
                    indexVertex.insert({vertexID, point});
                    vertexID += 1;
                }
                faceCoords.push_back(vertexIndex[vectorPoint]);
            }
        }
        vertices.resize(vertexIndex.size());

        for(vector<seidel::Point> faceCoords : faceTriangles){
            shared_ptr<Face> face = make_shared<Face>(Face(faceID++));
            shared_ptr<HalfEdge> previous = nullptr;
            shared_ptr<Vertex> firstVertex;
            shared_ptr<HalfEdge> firstHalfEdge;
            for(seidel::Point point: faceCoords){
                Vector3f position = point.position;
                int vertexNum =vertexIndex[{position.x(), position.y(), position.z()}];
                shared_ptr<HalfEdge> halfEdge = make_shared<HalfEdge>(HalfEdge(faceID++));
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
    for(shared_ptr<HalfEdge> halfEdge : halfEdges){
        shared_ptr<HalfEdge> twin = halfEdge->twin.lock();
        if(!twin){
            int origin = halfEdge->vertex->id;
            int destination = halfEdge->next->vertex->id;
            for (shared_ptr<HalfEdge> twin : halfEdgesByDestination[origin]){
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
