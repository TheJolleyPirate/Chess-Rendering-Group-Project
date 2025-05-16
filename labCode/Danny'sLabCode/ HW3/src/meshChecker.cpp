#include <include/mesh.hpp>
#include <include/meshChecker.hpp>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <queue>
#include <algorithm>
#include <assert.h>
#include <unordered_set>
using namespace std;
using namespace Eigen;

//by Arafat Hasan and Quonux on stack overflow
//https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
std::vector<std::string> split (const std::string &s, char delim) {
    std::vector<std::string> result;
    std::stringstream ss (s);
    std::string item;

    while (getline (ss, item, delim)) {
        result.push_back (item);
    }

    return result;
}

//loadDubiousOBJ: 2 loops fileloop and loop through vertices in faces
//number of vertices per face is trivial therefore, o(n)
tuple<vector<Vector3f>, vector<vector<int>>, int> loadDubiousOBJ(const string& filepath) {
    string line;
    ifstream file(filepath);
    vector<Vector3f> displayVertices;
    vector<vector<int>> displayFaces;
    int meshType = 0;
    while (getline(file, line)) {
        istringstream iss(line);
        string prefix;
        iss >> prefix;
        if ("v" == prefix) {
            Vector3f vertex;
            iss >> vertex[0] >> vertex[1] >> vertex[2];
            displayVertices.push_back(vertex);
        } else if ("f" == prefix) {
            vector<int> face;
            for(string vertData; iss >> vertex
            if(face.size() > meshType){
                meshType = face.size();
            }
            displayFaces.push_back(face);
        }
    }
    return make_tuple(displayVertices, displayFaces, meshType);
}

//convertDubiousOBJtoMesh: 2 nested loops loops through faces, and then vertices with that face.
//number of vertices per face is probably 3 or 4, therefore o(4n) = o(n)
tuple<vector<shared_ptr<Vertex>>, vector<shared_ptr<Face>>, vector<shared_ptr<HalfEdge>>, vector<vector<shared_ptr<HalfEdge>>>> 
        convertDubiousOBJtoMesh(vector<Vector3f> displayVertices, vector<vector<int>> displayFaces) {
    vector<shared_ptr<Vertex>> vertices(displayVertices.size(), nullptr);
    vector<shared_ptr<Face>> faces;
    vector<shared_ptr<HalfEdge>> halfEdges;
    int faceIdent, heIdent;
    faceIdent = heIdent = 0;
    vector<vector<shared_ptr<HalfEdge>>> heDest;
    heDest.resize(displayVertices.size());
    for(vector<int> faceCoords : displayFaces){
        shared_ptr<Face> face = make_shared<Face>(faceIdent++);
        //set face colour
        face->color = {255, 255, 255}; 
        shared_ptr<HalfEdge> previous = nullptr;
        shared_ptr<Vertex> firstVert;
        shared_ptr<HalfEdge> firstHE;
        for(int vertNum: faceCoords){
            vertNum -= 1;
            shared_ptr<HalfEdge> he = make_shared<HalfEdge>(heIdent++);
            if(previous != nullptr){
                previous->next = he;
                heDest[vertNum].push_back(previous);
            }
            else{
                firstHE = he;
            }
            shared_ptr<Vertex> v;
            if(vertices[vertNum] == nullptr){
                v = make_shared<Vertex>(vertNum);
                v->pos = displayVertices[vertNum];
                v->he = he;
                vertices[vertNum] = v;
            }
            else{
                v = vertices[vertNum];
            }
            he->vertex = v;
            he->face = face;
            he->twin = nullptr;
            halfEdges.push_back(he);
            previous = he;
        }
        previous->next = firstHE;
        heDest[firstHE->vertex->id].push_back(previous);
        face->he = firstHE;
        faces.push_back(face);
    }
    for(shared_ptr<HalfEdge> he : halfEdges){
        if(!he->has_twin()){
            int origion = he->vertex->id;
            int dest = he->next->vertex->id;
            for (shared_ptr<HalfEdge> twin : heDest[origion]){
                if(twin->vertex->id == dest && !twin->has_twin()){
                    he->twin = twin;
                    twin->twin = he;
                    break;
                }
            }
        }
    }
    for(int i = 0; i < vertices.size(); ++i){
        if(vertices[i] == nullptr){
            vertices[i] = make_shared<Vertex>(i);
            vertices[i]->pos = displayVertices[i];
        }
    }
    return make_tuple(vertices, faces, halfEdges, heDest);
}

//a version of this function which is safe for non-manifold meshes
vector<shared_ptr<Vertex>> neighbourVertices(const shared_ptr<Vertex> &vert) {
    vector<shared_ptr<Vertex>> neighbourhood;
    if(vert == nullptr || vert->he == nullptr){
        return neighbourhood;
    }
    shared_ptr<HalfEdge> he = vert->he;
    do {
        if(he == nullptr || he->twin == nullptr || he->twin->vertex == nullptr){
            break;
        }
        neighbourhood.push_back(he->twin->vertex);
        he = he->twin->next;
    }
    while(he != vert->he);
    return neighbourhood; 
}

//traverseVertex: recurrsive loop, o(x^n)
void traverseVertex(const shared_ptr<Vertex> &vert, unordered_set<int> &visited){
    if(vert == nullptr || visited.count(vert->id) > 0){
        return;
    }
    visited.insert(vert->id);
    vector<shared_ptr<Vertex>> neighbourhood = neighbourVertices(vert);
    for(shared_ptr<Vertex> neighbour : neighbourhood){
        traverseVertex(neighbour, visited);
    }
    return;
}

//meshConnected: recurrsive loop, o(n^n)
bool meshConnected(const vector<shared_ptr<Vertex>> &vertices){
    unordered_set<int> visited;
    if(vertices[0] == nullptr){
        return false;
    }

    stack<shared_ptr<Vertex>> stk;
    stk.push(vertices[0]);

    while (!stk.empty()) {
        shared_ptr<Vertex> vert = stk.top(); 
        stk.pop();
        if (vert == nullptr || visited.count(vert->id) > 0){
            continue;
        }
        visited.insert(vert->id);
        for (std::shared_ptr<Vertex> neighbour : neighbourVertices(vert)) {
            stk.push(neighbour);
        }
    }

    traverseVertex(vertices[0], visited);
    int numVisited = visited.size();
    int numVertices = vertices.size();
    if(numVisited == numVertices){
        return true;
    }
    return false;
}

//mesh closed: 1 loop, o(n)
bool meshClosed(const vector<shared_ptr<HalfEdge>> &halfEdges){
    for(shared_ptr<HalfEdge> he : halfEdges){
        if(!he->face->exists){
            return false;
        }
        if(!he->has_twin()){
            return false;
        }
        if(!he->twin->has_twin() || he->twin->twin->id != he->id){
            return false;
        }
    }
    return true;
}

//meshManifold: 2 nested loops at start, then another 2 and another, o(n^2)
bool meshManifold(const vector<shared_ptr<Face>> &faces, const vector<shared_ptr<HalfEdge>> &halfEdges, const vector<shared_ptr<Vertex>> &vertices){
    //to check edge manifoldness, make sure their are exactly 2 faces on each edge.
    //first check to make sure only 2 faces share an edge
    vector<vector<int>> numEdgesBetweenVertices(vertices.size(), vector<int>(vertices.size(), 0));
    for(shared_ptr<Face> face : faces){
        shared_ptr<HalfEdge> current = face->he;
        int startID = current->id;
        do{
            if(current->vertex == nullptr || current->next == nullptr || current->next->vertex == nullptr){
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
    vector<vector<int>> startingPoints(vertices.size(), vector<int>());
    //then check if there are any edges with no face
    for(shared_ptr<HalfEdge> he : halfEdges){
        if(!he->face->exists){
            return false;
        }
        int id = he->vertex->id;
        startingPoints[id].push_back(he->id);
    }
    auto getPrevious = [](shared_ptr<HalfEdge> c){
        //iterates around polygon untill getting back to start
        shared_ptr<HalfEdge> previous;
        shared_ptr<HalfEdge> he;
        he = c;
        while(true){
            previous = he;
            he = he->next;
            if(he->vertex->id == c->vertex->id){
                break;
            }
        }
        return previous;
    };
    //check vertex manifoldness by exploring vertex fans
    int index = 0;
    int maxFan = vertices.size();

    for(shared_ptr<Vertex> v : vertices){
        vector<int> currentFan;
        if(v == nullptr || v->he == nullptr){
            continue;
        }
        shared_ptr<HalfEdge> he = v->he;
        //get a halfedge on a random fan of the current vertex
        //get all halfedges which are part of the fan by traverseing it
        //first forward
        bool fullyExplored = false;
        while(true){
            if(find(currentFan.begin(), currentFan.end(), he->id) != currentFan.end()){
                fullyExplored = true;
                break;
            }
            currentFan.push_back(he->id);
            if(currentFan.size() > maxFan){
                return false;
            }
            if(he->twin == nullptr || he->twin->face == nullptr || he->twin->next == nullptr){
                break;
            }
            he = he->twin->next;
        }

        //then backwards
        he = getPrevious(v->he);
        if(!fullyExplored && he->twin->face != nullptr){
            if(he->twin == nullptr){
                break;
            }
            he = he->twin;
            while(true){
                if(find(currentFan.begin(), currentFan.end(), he->id) != currentFan.end()){
                    break;
                }
                currentFan.push_back(he->id);
                if(currentFan.size() > maxFan){
                    return false;
                }
                he = getPrevious(he);
                if(he->twin == nullptr || he->twin->face == nullptr || he->twin->next == nullptr){
                    break;
                }
                he = he->twin;
            }
        }
        //if there is a half edge coming from this vertex
        //which has not been found in the fan traversal
        //then the vertex is not manifold
        for(int id : startingPoints[v->id]){
            if(find(currentFan.begin(), currentFan.end(), id) == currentFan.end()){
                return false;
            }
        }
    }
    return true;
}

//meshOrientation: 1 loop, o(n)
bool meshOrientation(const vector<shared_ptr<HalfEdge>> &halfEdges, const int numVertices){
    //two adjacent polygons have a consistant orientation if touching edges face opposite directions
    
    //make a bool for each origin vertex - destination vertex combo and set it to false
    vector<vector<bool>> originToDest(numVertices, vector<bool>(numVertices, false));
    for(shared_ptr<HalfEdge> he : halfEdges){
        int origin = he->vertex->id;
        int destination = he->next->vertex->id;
        //if the current half edge's origin and destination have been encountered return false
        if(originToDest[origin][destination]){
            return false;
        }
        //set the current origin and destination combo to true
        originToDest[origin][destination] = true;
    }
    return true;
}

//makeConsistentOrientation: recurrsive loop o(n^n)
vector<shared_ptr<Face>> makeConsistentOrientation(const vector<shared_ptr<Face>> &faces, const int numVertices){
    vector<shared_ptr<Face>> consistentFaces;
    unordered_set<int> visited;
    //get a matrix of what faces connect to what edges
    vector<vector<vector<shared_ptr<Face>>>> edges(numVertices, vector<vector<shared_ptr<Face>>>(numVertices));
    for(shared_ptr<Face> face : faces){
        shared_ptr<HalfEdge> he = face->he;
        shared_ptr<HalfEdge> previous;
        int startID = he->id;
        do{
            previous = he;
            he = he->next;
            int v1 = previous->vertex->id;
            int v2 = he->vertex->id;
            edges[v1][v2].push_back(face);
            edges[v2][v1].push_back(face);
        }
        while(he->id != startID);
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
        shared_ptr<HalfEdge> current = face->he;
        int startID = current->id;
        do {
            HalfEdge newHalfEdge = *current;
            clones.push_back(make_shared<HalfEdge>(newHalfEdge));
            current = current->next;
        } 
        while (current->id != startID);

        int n = clones.size();
        for(int i = 0; i < n; ++i){
            clones[i]->twin = nullptr;
            if(reverse){
                clones[i]->next = clones[(i - 1 + n) % n];
            }
            else{
                clones[i]->next = clones[(i + 1) % n];
            }
            directions.push_back({clones[i]->vertex->id, clones[i]->next->vertex->id});
        }
        face->he = clones[0];
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
                current = adj->he;
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
                shared_ptr<HalfEdge> current = face->he;
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
    return consistentFaces;
}

class Node{
    public:
        enum NodeType{
            ROOT,
            VERTEX,
            EDGE,
            TRAPAZOID,
        };
        enum NodeType nodeType;
        shared_ptr<Vertex> vertex; //the vertex this represents if nodetype is vertex
        shared_ptr<HalfEdge> edge; //the edge this represents if nodetype is edge
        shared_ptr<Trapazoid> trapazoid; //the trapazoid this represents if nodetype is trapazoid
        const int id; //unique node id
        shared_ptr<Node> left; //left child node
        shared_ptr<Node> right; //right child node
        shared_ptr<Node> parent; //parent node
        Node(const int id): id(id){
            this->vertex = nullptr;
            this->trapazoid = nullptr;
            this->edge = nullptr;
        }
};

class Trapazoid{
    public:
        vector<shared_ptr<Trapazoid>> up; //trapazoids directly above this one
        vector<shared_ptr<Trapazoid>> down; //trapazoids directly below this one
        shared_ptr<HalfEdge> lseg; //left edge
        shared_ptr<HalfEdge> rseg; //right edge
        shared_ptr<Node> sink; //Position of trapezoid in tree structure
        bool validState; //Represents validity of trapezoid (Inside or outside)
        const int id; //unique ID for this Trapazoid
        shared_ptr<Ray> highRay; //the ray which bounds the top of the trapazoid
        shared_ptr<Ray> lowRay; //the ray which bounds the bottom of the trapazoid
        Trapazoid(shared_ptr<Node> node, shared_ptr<Ray> lowRay, shared_ptr<Ray> highRay, int id): id(id){
            this->sink = node->parent;
            this->highRay = highRay;
            this->lowRay = lowRay;
            setLeftEdge(node);
            setRightEdge(node);
        }
        void setLeftEdge(const shared_ptr<Node> &node){
            int previous = node->id;
            shared_ptr<Node> current = node->parent;
            shared_ptr<HalfEdge> edge = nullptr;
            if(current == nullptr){
                return;
            }
            while(true){
                if(current->nodeType == Node::EDGE){
                    if(current->left == nullptr || current->left->id != previous){
                        edge = current->edge;
                        break;
                    }
                }
                if(current->parent == nullptr){
                    return;
                }
                previous = current->id;
                current = current->parent;
            };
            this->lseg = edge;
        }
        void setRightEdge(const shared_ptr<Node> &node){
            int previous = node->id;
            shared_ptr<Node> current = node->parent;
            shared_ptr<HalfEdge> edge = nullptr;
            if(current == nullptr){
                return;
            }
            while(true){
                if(current->nodeType == Node::EDGE){
                    if(current->right == nullptr || current->right->id != previous){
                        edge = current->edge;
                        break;
                    }
                }
                if(current->parent == nullptr){
                    return;
                }
                previous = current->id;
                current = current->parent;
            };
            this->rseg = edge;
        }
};

class Ray{
    public:
        shared_ptr<Vertex> start; //origin vertex of ray
        Vector3f leftEnd; //point ray terminates in -x direction
        Vector3f rightEnd; //point ray terminates in +x direction
        float yValue; //the y value of the ray
        Ray(shared_ptr<Vertex> start, vector<shared_ptr<HalfEdge>> faceEdges){
            int yValue = start->pos.y();
            float rightX = FLT_MAX;
            Vector3f rightEnd = start->pos;
            float leftX = -FLT_MAX;
            Vector3f leftEnd = start->pos;
            for(shared_ptr<HalfEdge> he : faceEdges){
                Vector3f origin = he->vertex->pos;
                Vector3f dest = he->next->vertex->pos;
                //if yValue between origin and dest
                if((origin.y() > yValue && yValue > dest.y()) || (origin.y() < yValue && yValue < dest.y())){
                    float yOrigin = origin.y();
                    float yDest = dest.y();
                    float dy = yDest - yOrigin;
                    if(std::abs(dy) < 1e-8f){
                        if(yValue == yOrigin || yValue == yDest){
                            if(origin.x() < dest.x()){
                                if(origin.x() > start->pos.x() && origin.x() < rightX){
                                    rightEnd = origin;
                                }
                                if(dest.x() < start->pos.x() && dest.x() > leftX){
                                    leftEnd = dest;
                                }
                            }
                        }
                    }
                    else{
                        float mult = (yValue - yOrigin) / dy;
                        Vector3f intercept = origin + (mult * (dest - origin));
                        if(intercept.x() > start->pos.x() && intercept.x() < rightX){
                            rightEnd = intercept;
                        }
                        if(intercept.x() < start->pos.x() && intercept.x() > leftX){
                            leftEnd = intercept;
                        }
                    }
                }
            }
            this->start = start;
            this->yValue = yValue;
            this->rightEnd = rightEnd;
            this->leftEnd = leftEnd;
        }
};

//triangulateQuad: o(1)
vector<vector<Vector3f>> triangulateQuad(Vector3f leftOrigin, Vector3f leftDest, Vector3f rightOrigin, Vector3f rightDest){
    vector<vector<Vector3f>> triangles;
    //maintain orientation
    //if leftDest higher then left origin then the orientation is clockwise
    if(leftDest.y() > leftOrigin.y()){
        //check if already triangle
        if(leftDest == rightOrigin){
            Vector3f A = leftOrigin;
            Vector3f B = leftDest;
            Vector3f C = rightDest;
            triangles.push_back({A, B, C});
            return triangles;
        }
        else if(rightDest == leftOrigin){
            Vector3f A = leftOrigin;
            Vector3f B = leftDest;
            Vector3f C = rightOrigin;
            triangles.push_back({A, B, C});
            return triangles;
        }
        //form triangles from vertices
        Vector3f A1 = leftOrigin;
        Vector3f B1 = leftDest;
        Vector3f C1 = rightDest;
        triangles.push_back({A1, B1, C1});

        Vector3f A2 = leftDest;
        Vector3f B2 = rightOrigin;
        Vector3f C2 = rightDest;
        triangles.push_back({A2, B2, C2});
    }
    //else anticlockwise
    else{
        if(leftOrigin == rightDest){
            Vector3f A = leftOrigin;
            Vector3f B = leftDest;
            Vector3f C = rightOrigin;
            triangles.push_back({A, B, C});
            return triangles;
        }
        else if(rightOrigin == leftDest){
            Vector3f A = leftOrigin;
            Vector3f B = leftDest;
            Vector3f C = rightDest;
            triangles.push_back({A, B, C});
            return triangles;
        }
        Vector3f A1 = leftOrigin;
        Vector3f B1 = leftDest;
        Vector3f C1 = rightDest;
        triangles.push_back({A1, B1, C1});

        Vector3f A2 = leftDest;
        Vector3f B2 = rightOrigin;
        Vector3f C2 = rightDest;
        triangles.push_back({A2, B2, C2});
    }
    return triangles;
}

//Seidel's algorithm has a big o of O(n log*n)
vector<vector<Vector3f>> triangulate(const shared_ptr<Face> &face){
    auto tieBreak = [](const Vector3f a, const Vector3f b) -> Vector3f{
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
    };
    
    vector<vector<Vector3f>> triangles;
    int trapID = 0;
    int nodeID = 0;
    //I use SEIDEL'S algorithm to perform trapezoidation
    //source: http://www.polygontriangulation.com/2018/07/triangulation-algorithm.html
    vector<shared_ptr<HalfEdge>> faceEdges;
    shared_ptr<HalfEdge> currentHalfedge = face->he;
    int startID = currentHalfedge->id;
    do{
        faceEdges.push_back(currentHalfedge);
        shared_ptr<HalfEdge> previous = currentHalfedge;
        currentHalfedge = currentHalfedge->next;
    }
    while(currentHalfedge->id != startID);
    
    shared_ptr<Node> tree;
    //get arbitrary edge from list of edges
    for(shared_ptr<HalfEdge> edge : faceEdges){
        auto processVertexNode = [&](const shared_ptr<Vertex> &vert) -> shared_ptr<Node>{
            if(tree == nullptr){
                //nodeType only root when tree is empty
                shared_ptr<Ray> ray = make_shared<Ray>(vert, faceEdges);
                tree = make_shared<Node>(nodeID++);
                tree->nodeType = Node::VERTEX;
                tree->vertex = vert;

                tree->left = make_shared<Node>(Node(nodeID++));
                tree->left->parent = tree;
                std::shared_ptr<Trapazoid> bottomTrap = make_shared<Trapazoid>(tree->left, nullptr, ray, trapID++);
                tree->left->nodeType = Node::TRAPAZOID;
                tree->left->trapazoid = bottomTrap;

                tree->right = make_shared<Node>(nodeID++);
                tree->right->parent = tree;
                std::shared_ptr<Trapazoid> topTrap = make_shared<Trapazoid>(tree->right, ray, nullptr, trapID++);
                tree->right->nodeType = Node::TRAPAZOID;
                tree->right->trapazoid = topTrap;
                topTrap->down.push_back(bottomTrap);
                bottomTrap->up.push_back(topTrap);
                return tree;
            }
            shared_ptr<Node> currentNode = tree;
            while(true){
                if(currentNode->nodeType == Node::VERTEX){
                    Vector3f a = vert->pos;
                    Vector3f b = currentNode->vertex->pos;
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
                    Vector3f origin = e->vertex->pos;
                    Vector3f dest = e->next->vertex->pos;
                    float yValue = vert->pos.y();
                    Vector3f unitVector = (dest - origin).normalized();
                    float mult = (yValue - origin.y()) / unitVector.y();
                    Vector3f intercept = origin + (unitVector * mult);
                    Vector3f a = vert->pos;
                    Vector3f b = intercept;
                    if(a.x() > b.x() || (a.x() == b.x() && tieBreak(a, b) == a)){
                        currentNode = currentNode->right;
                    }
                    else{
                        currentNode = currentNode->left;
                    }
                }
                else{
                    //if not vertex or edge must be trapazoid
                    shared_ptr<Trapazoid> old = currentNode->trapazoid;
                    shared_ptr<Ray> ray = make_shared<Ray>(vert, faceEdges);
                    shared_ptr<Ray> oldHighRay = old->highRay;
                    shared_ptr<Ray> oldLowRay = old->lowRay;
                    currentNode->nodeType = Node::VERTEX;
                    currentNode->trapazoid = nullptr;
                    currentNode->vertex = vert;
                    currentNode->left = make_shared<Node>(nodeID++);
                    currentNode->left->parent = currentNode;
                    std::shared_ptr<Trapazoid> bottomTrap = make_shared<Trapazoid>(currentNode->left, oldLowRay, ray, trapID++);
                    currentNode->left->nodeType = Node::TRAPAZOID;
                    currentNode->left->trapazoid = bottomTrap;
                    currentNode->right = make_shared<Node>(nodeID++);
                    currentNode->right->parent = currentNode;
                    std::shared_ptr<Trapazoid> topTrap = make_shared<Trapazoid>(currentNode->right, ray, oldHighRay, trapID++);
                    currentNode->right->nodeType = Node::TRAPAZOID;
                    currentNode->right->trapazoid = topTrap;
                    topTrap->up = old->up;
                    bottomTrap->down = old->down;
                    topTrap->down.push_back(bottomTrap);
                    bottomTrap->up.push_back(topTrap);
                    for(shared_ptr<Trapazoid> above : topTrap->up){
                        replace(above->down.begin(), above->down.end(), old, topTrap);
                    }
                    for(shared_ptr<Trapazoid> below : bottomTrap->down){
                        replace(below->up.begin(), below->up.end(), old, bottomTrap);
                    }
                    return currentNode;
                }
            }
        };
        auto processEdgeNode = [&](const shared_ptr<HalfEdge> &ed, const shared_ptr<Node> &lower, const shared_ptr<Node> &upper) -> void{
            auto insertEdge = [&](const shared_ptr<Node> &node){
                shared_ptr<Trapazoid> old = node->trapazoid;
                shared_ptr<Ray> highRay = old->highRay;
                shared_ptr<Ray> lowRay = old->lowRay;
                node->nodeType = Node::EDGE;
                node->trapazoid = nullptr;
                node->edge = ed;
                node->left = make_shared<Node>(nodeID++);
                node->left->parent = node;
                std::shared_ptr<Trapazoid> bottomTrap = make_shared<Trapazoid>(node->left, lowRay, highRay, trapID++);
                node->left->nodeType = Node::TRAPAZOID;
                node->left->trapazoid = bottomTrap;
                node->right = make_shared<Node>(Node(nodeID++));
                node->right->parent = node;
                std::shared_ptr<Trapazoid> topTrap = make_shared<Trapazoid>(node->right, lowRay, highRay, trapID++);
                node->right->nodeType = Node::TRAPAZOID;
                node->right->trapazoid = topTrap;
                Vector3f a;
                Vector3f b;
                vector<shared_ptr<Node>> children = {node->left, node->right};
                //get new ups and downs, and set neighbours
                for(shared_ptr<Trapazoid> above : old->up){
                    auto iter = find(above->down.begin(), above->down.end(), old);
                    if ( iter != above->down.end() ) {
                        above->down.erase(iter);
                    }
                    //get lower left above x coord
                    float lowerLeftAbove;
                    if(above->lseg == nullptr){
                        lowerLeftAbove = -FLT_MAX;
                    }
                    else{
                        a = above->lseg->vertex->pos;
                        b = above->lseg->next->vertex->pos;
                        if(a.y() < b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                            lowerLeftAbove = a.x();
                        }
                        else{
                            lowerLeftAbove = b.x();
                        }
                    }

                    //get lower right above x coord
                    float lowerRightAbove;
                    if(above->rseg == nullptr){
                        lowerRightAbove = FLT_MAX;
                    }
                    else{
                        a = above->rseg->vertex->pos;
                        b = above->rseg->next->vertex->pos;
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
                        if(child->trapazoid->lseg == nullptr){
                            upperLeftChild = -FLT_MAX;
                        }
                        else{
                            a = child->trapazoid->lseg->vertex->pos;
                            b = child->trapazoid->lseg->next->vertex->pos;
                            if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                                upperLeftChild = a.x();
                            }
                            else{
                                upperLeftChild = b.x();
                            }
                        }
                        //get upper right child x coord
                        float upperRightChild;
                        if(child->trapazoid->rseg == nullptr){
                            upperRightChild = FLT_MAX;
                        }
                        else{
                            a = child->trapazoid->rseg->vertex->pos;
                            b = child->trapazoid->rseg->next->vertex->pos;
                            if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                                upperRightChild = a.x();
                            }
                            else{
                                upperRightChild = b.x();
                            }
                        }
                        if(lowerLeftAbove < upperRightChild && lowerRightAbove > upperLeftChild){
                            above->down.push_back(child->trapazoid);
                        }
                    }
                }
                for(shared_ptr<Trapazoid> below : old->down){
                    auto iter = find(below->up.begin(), below->up.end(), old);
                    if ( iter != below->up.end() ) {
                        below->up.erase(iter);
                    }
                    //get upper left below x coord
                    float upperLeftBelow;
                    if(below->lseg == nullptr){
                        upperLeftBelow = -FLT_MAX;
                    }
                    else{
                        a = below->lseg->vertex->pos;
                        b = below->lseg->next->vertex->pos;
                        if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                            upperLeftBelow = a.x();
                        }
                        else{
                            upperLeftBelow = b.x();
                        }
                    }
                    //get upper right below x coord
                    float upperRightBelow;
                    if(below->rseg == nullptr){
                        upperRightBelow = FLT_MAX;
                    }
                    else{
                        a = below->rseg->vertex->pos;
                        b = below->rseg->next->vertex->pos;
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
                        if(child->trapazoid->lseg == nullptr){
                            lowerLeftChild = -FLT_MAX;
                        }
                        else{
                            a = child->trapazoid->lseg->vertex->pos;
                            b = child->trapazoid->lseg->next->vertex->pos;
                            if(a.y() < b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                                lowerLeftChild = a.x();
                            }
                            else{
                                lowerLeftChild = b.x();
                            }
                        }
                        //get lower right child x coord
                        float lowerRightChild;
                        if(child->trapazoid->rseg == nullptr){
                            lowerRightChild = FLT_MAX;
                        }
                        else{
                            a = child->trapazoid->rseg->vertex->pos;
                            b = child->trapazoid->rseg->next->vertex->pos;
                            if(a.y() < b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
                                lowerRightChild = a.x();
                            }
                            else{
                                lowerRightChild = b.x();
                            }
                        }
                        if(upperLeftBelow < lowerRightChild && upperRightBelow > lowerLeftChild){
                            below->up.push_back(child->trapazoid);
                        }
                    }
                }
            };

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
                    Vector3f o = ed->vertex->pos;
                    Vector3f n = ed->next->vertex->pos;
                    if(o.y() > n.y() || (o.y() == n.y() && tieBreak(o, n) == o)){
                        targetTop = ed->vertex;
                    }
                    else{
                        targetTop = ed->next->vertex;
                    }
                    shared_ptr<Vertex> currentTop;
                    o = currentNode->edge->vertex->pos;
                    n = currentNode->edge->next->vertex->pos;
                    if(o.y() > n.y() || (o.y() == n.y() && tieBreak(o, n) == o)){
                        currentTop = currentNode->edge->vertex;
                    }
                    else{
                        currentTop = currentNode->edge->next->vertex;
                    }
                    Vector3f a = targetTop->pos;
                    Vector3f b = currentTop->pos;
                    if(a.x() > b.x() || (a.x() == b.x() && tieBreak(a, b) == a)){
                        currentNode = currentNode->right;
                    }
                    else{
                        currentNode = currentNode->left;
                    }
                }
                else{
                    placeInserted = currentNode->id;
                    insertEdge(currentNode);
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
                    Vector3f o = ed->vertex->pos;
                    Vector3f n = ed->next->vertex->pos;
                    if(o.y() < n.y() || (o.y() == n.y() && tieBreak(o, n) == o)){
                        targetBottom = ed->vertex;
                    }
                    else{
                        targetBottom = ed->next->vertex;
                    }
                    shared_ptr<Vertex> currentBottom;
                    o = currentNode->edge->vertex->pos;
                    n = currentNode->edge->next->vertex->pos;
                    if(o.y() < n.y() || (o.y() == n.y() && tieBreak(o, n) == o)){
                        currentBottom = currentNode->edge->vertex;
                    }
                    else{
                        currentBottom = currentNode->edge->next->vertex;
                    }
                    Vector3f a = targetBottom->pos;
                    Vector3f b = currentBottom->pos;
                    if(a.x() > b.x() || (a.x() == b.x() && tieBreak(a, b) == a)){
                        currentNode = currentNode->right;
                    }
                    else{
                        currentNode = currentNode->left;
                    }
                }
                else{
                    insertEdge(currentNode);
                    break;
                }
            }
        };
        //get edge vertices
        shared_ptr<Vertex> v1 = edge->vertex;
        shared_ptr<Vertex> v2 = edge->next->vertex;
        //find higher and lower vertex
        shared_ptr<Vertex> higher;
        shared_ptr<Vertex> lower;
        Vector3f a = v1->pos;
        Vector3f b = v2->pos;
        if(a.y() > b.y() || (a.y() == b.y() && tieBreak(a, b) == a)){
            higher = v1;
            lower = v2;
        }
        else{
            higher = v2;
            lower = v1;
        }
        //traverse tree and place higher vertex and lower vertex
        shared_ptr<Node> upperNode = processVertexNode(higher);
        shared_ptr<Node> lowerNode = processVertexNode(lower);
        //place edge below upper node but above lower node
        processEdgeNode(edge, lowerNode, upperNode);
    }
    //make sure all nodes are up to date and get list of trapazoids
    vector<shared_ptr<Trapazoid>> trapazoids;
    auto populateTrapazoids = [&](const auto &self,const shared_ptr<Node> &node) -> void{
        if(node->nodeType == Node::TRAPAZOID){
            node->trapazoid->setLeftEdge(node);
            node->trapazoid->setRightEdge(node);
            trapazoids.push_back(node->trapazoid);
            return;
        }
        else{
            self(self, node->left);
            self(self, node->right);
            return;
        }
    };
    populateTrapazoids(populateTrapazoids, tree);

    //check which trapazoids are valid
    auto inPolygon = [](const shared_ptr<Trapazoid> trap) -> bool{
        auto findAdjacentTrapazoid = [&](const auto &self, shared_ptr<Node> currentNode, bool directionLeft, int &counter) -> void{
            bool subDirectionLeft = true;
            //get node according to direction
            if(directionLeft){
                currentNode = currentNode->left;
            }
            else{
                currentNode = currentNode->right;
            }
            //find the closest trapazoid from that node
            while(true){
                if(currentNode->nodeType == Node::TRAPAZOID){
                    counter += 1;
                    //check if tree outside polygon
                    if(currentNode->trapazoid->lseg != nullptr && currentNode->trapazoid->rseg != nullptr){
                        //get segment according to direction
                        while(true){
                            int previous = currentNode->id;
                            currentNode = currentNode->parent;
                            if(currentNode == nullptr){
                                return;
                            }
                            if(currentNode->nodeType == Node::EDGE){
                                if(directionLeft && currentNode->right != nullptr && currentNode->right->id == previous){
                                    break;
                                }
                                else if(!directionLeft && currentNode->left != nullptr && currentNode->left->id == previous){
                                    break;
                                }
                            }
                        }
                        self(self, currentNode, directionLeft, counter);
                    }
                    break;
                }
                //we are trying to move just one spot in direction
                else if(currentNode->nodeType == Node::EDGE){
                    if(directionLeft){
                        currentNode = currentNode->right;
                    }
                    else{
                        currentNode = currentNode->left;
                    }
                }
                //this works due to the fact that higher nodes are added first and are thus higher in the tree
                else if(currentNode->nodeType == Node::VERTEX){
                    if(subDirectionLeft){
                        currentNode = currentNode->left;
                        subDirectionLeft = false;
                    }
                    else{
                        currentNode = currentNode->right;
                        subDirectionLeft = true;
                    }
                }
            }
        };
        //get current node from trapazoid
        shared_ptr<Node> currentNode = trap->sink;
        //traverse upward until left or right edge found
        bool directionLeft;
        while(true){
            int previous = currentNode->id;
            currentNode = currentNode->parent;
            if(currentNode == nullptr){
                return false;
            }
            if(currentNode->nodeType == Node::EDGE){
                if(currentNode->right != nullptr && currentNode->right->id == previous){
                    directionLeft = true;
                }
                else{
                    directionLeft = false;
                }
                break;
            }
        }
        int counter = 0;
        findAdjacentTrapazoid(findAdjacentTrapazoid, currentNode, directionLeft, counter);
        if(counter % 2 == 0){
            return false;
        }
        else{
            return true;
        }
    };
    for(shared_ptr<Trapazoid> trap : trapazoids){
        if(trap->lseg == nullptr || trap->rseg == nullptr){
            trap->validState = false;
            continue;
        }
        if(inPolygon(trap)){
            trap->validState = true;
        }
        else{
            trap->validState = false;
        }
    }

    //transform trapazoids into triangles
    for(shared_ptr<Trapazoid> trap : trapazoids){
        if(!trap->validState){
            continue;
        }
        //get vertices of trapazoid
        //use rays and side segments to interporlate the four vertices

        auto interpolate = [](Vector3f origin, Vector3f dest, float yIntercept, bool leftSide) -> Vector3f{
            float yOrigin = origin.y();
            float yDest = dest.y();
            float dy = yDest - yOrigin;
            if(std::abs(dy) < 1e-8f){
                if(yIntercept == yOrigin || yIntercept == yDest){
                    if(origin.x() < dest.x()){
                        if(leftSide){
                            return dest;
                        }
                        else{
                            return origin;
                        }
                    }
                    else{
                        if(leftSide){
                            return origin;
                        }
                        else{
                            return dest;
                        }
                    }
                }
            }
            else{
                float mult = (yIntercept - yOrigin) / dy;
                Vector3f intercept = origin + (mult * (dest - origin));
                return intercept;
            }
            throw yIntercept;
        };

        Vector3f leftOrigin;
        Vector3f leftDest;
        Vector3f rightOrigin;
        Vector3f rightDest;
        float highY = trap->highRay->yValue;
        float lowY = trap->lowRay->yValue;
        Vector3f leftEdgeOrigin = trap->lseg->vertex->pos;
        Vector3f leftEdgeDest = trap->lseg->next->vertex->pos;
        Vector3f rightEdgeOrigin = trap->rseg->vertex->pos;
        Vector3f rightEdgeDest = trap->rseg->next->vertex->pos;
        
        Vector3f leftLowVertex;
        Vector3f leftHighVertex;
        Vector3f rightLowVertex;
        Vector3f rightHighVertex;
        try{
            leftLowVertex = interpolate(leftEdgeOrigin, leftEdgeDest, lowY, true);
            leftHighVertex = interpolate(leftEdgeOrigin, leftEdgeDest, highY, true);
            rightLowVertex = interpolate(rightEdgeOrigin, rightEdgeDest, lowY, false);
            rightHighVertex = interpolate(rightEdgeOrigin, rightEdgeDest, highY, false);
        }
        catch(float yIntercept){
            //bad trapazoid invalid y intercept
            continue;
        }

        if(leftEdgeOrigin.y() > leftEdgeDest.y() || tieBreak(leftEdgeOrigin, leftEdgeDest) == leftEdgeOrigin){
            leftOrigin = leftHighVertex;
            leftDest = leftLowVertex;
        }
        else{
            leftOrigin = leftLowVertex;
            leftDest = leftHighVertex;
        }
        if(rightEdgeOrigin.y() > rightEdgeDest.y() || tieBreak(rightEdgeOrigin, rightEdgeDest) == rightEdgeOrigin){
            rightOrigin = rightHighVertex;
            rightDest = rightLowVertex;
        }
        else{
            rightOrigin = rightLowVertex;
            rightDest = rightHighVertex;
        }
        vector<vector<Vector3f>> currentTriangles = triangulateQuad(leftOrigin, leftDest, rightOrigin, rightDest);
        triangles.insert(triangles.end(), currentTriangles.begin(), currentTriangles.end());
    }
    
    return triangles;
}

//triangulate: looping through Seidel's algorithm, o(n^2 log*n)
//when tri or quad mesh: o(1)
vector<vector<vector<Vector3f>>> triangulate(const vector<shared_ptr<Face>> &faces){
    vector<vector<vector<Vector3f>>> triangles;
    for(shared_ptr<Face> face : faces){
        //check how many sides current face has
        shared_ptr<HalfEdge> current = face->he;
        int startID = current->id;
        int numEdges = 0;
        do{
            current = current->next;
            numEdges += 1;
        }
        while(current->id != startID);
        vector<vector<Vector3f>> tri;
        if(numEdges == 3){
            tri.push_back({face->he->vertex->pos, face->he->next->vertex->pos, face->he->next->next->vertex->pos});
        }
        else if (numEdges == 4){
            //for left and right edges all that matters is they are opposite edges actual orientation doesn't matter
            Vector3f leftOrigin = face->he->vertex->pos;
            Vector3f leftDest = face->he->next->vertex->pos;
            Vector3f rightOrigin = face->he->next->next->vertex->pos;
            Vector3f rightDest = face->he->next->next->next->vertex->pos;
            tri = triangulateQuad(leftOrigin, leftDest, rightOrigin, rightDest);
        }
        else{
            tri = triangulate(face);
        }
        triangles.push_back(tri);
    }
    return triangles;
}

//nSidedVolume: looping through Seidel's algorithm, o(n^2 log*n)
//when tri or quad mesh: o(n) or o(n^n) if needs to be made consistent
float nSidedVolume(vector<shared_ptr<Face>> faces, const bool consistantOri, const int numVertices){
    //make sure faces are orientated consistently
    if(!consistantOri){
        try{
            faces = makeConsistentOrientation(faces, numVertices);
        }
        catch(runtime_error err){
            cerr << err.what() << "\n";
            return NAN;
        }
    }
    //use SEIDEL'S algorithm to triangulate faces
    vector<vector<vector<Vector3f>>> triangulatedFaces = triangulate(faces);
    //for each triangle calculate partial volume using 3d version of Shoelace theorm by
    //Cha Zhang and Tsuhan Chen
    float totalVolume = 0.0;
    for(vector<vector<Vector3f>> face : triangulatedFaces){
        for(vector<Vector3f> triangle : face){
            //formula: 1/6 A dot (B cross C)
            //formula: Vi = 1/6 (-x3y2z1 + x2y3z1 + x3y1z2 - x1y3z2 - x2y1z3 + x1y2z3)
            Vector3f v0 = triangle[0];
            Vector3f v1 = triangle[1];
            Vector3f v2 = triangle[2];
            float arg1 = -(v2.x() * v1.y() * v0.z());
            float arg2 = (v1.x() * v2.y() * v0.z());
            float arg3 = (v2.x() * v0.y() * v1.z());
            float arg4 = -(v0.x() * v2.y() * v1.z());
            float arg5 = -(v1.x() * v0.y() * v2.z());
            float arg6 = (v0.x() * v1.y() * v2.z());
            float partialVolume = (arg1 + arg2 + arg3 + arg4 + arg5 + arg6) / 6.0;
            totalVolume += partialVolume;
        }
    }
    totalVolume = std::abs(totalVolume);
    return totalVolume;
}

map<string, float> checkMesh(const string filepath){
    map<string, float> meshInfo;
    bool triangleMesh = false;
    auto[displayVertices, displayFaces, meshType] = loadDubiousOBJ(filepath);
    meshInfo["is_triangular"] = static_cast<bool>(meshType == 3);
    cout << "ran loadDubiousOBJ, meshType: " << meshType << "\n";
    auto[vertices, faces, halfEdges, heDest] = convertDubiousOBJtoMesh(displayVertices, displayFaces);
    cout << "ran convertDubiousOBJtoMesh, num polygons: " << faces.size() << "\n";
    meshInfo["is_single_part"] = meshConnected(vertices);
    cout << "ran meshConnected, is_single_part: " << meshInfo["is_single_part"] << "\n";
    meshInfo["is_closed"] = meshClosed(halfEdges);
    cout << "ran meshClosed, is_closed: " << meshInfo["is_closed"] << "\n";
    meshInfo["is_manifold"] = meshManifold(faces, halfEdges, vertices);
    cout << "ran meshManifold, is_manifold: " << meshInfo["is_manifold"] << "\n";
    meshInfo["is_consist_ori"] = meshOrientation(halfEdges, vertices.size());
    cout << "ran meshOrientation, is_consist_ori: " << meshInfo["is_consist_ori"] << "\n";
    meshInfo["polygon_volume"] = nSidedVolume(faces, meshInfo["is_consist_ori"], vertices.size());
    cout << "ran nSidedVolume, polygon_volume: " << meshInfo["polygon_volume"] << "\n";
    return meshInfo;
}