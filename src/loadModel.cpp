#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <map>
#include "Eigen/Dense"
#include <fstream>

#include "meshChecker.hpp"
#include "object.hpp"

using namespace std;
using namespace Eigen;

struct Mesh
    {
        // Default Constructor
        Mesh(){}
        // Variable Set Constructor
        Mesh(std::vector<Vertex>& _Vertices, std::vector<unsigned int>& _Indices){
            Vertices = _Vertices;
            Indices = _Indices;
        }
        // Vertex List
        std::vector<Vertex> Vertices;
        // Index List
        std::vector<unsigned int> Indices;
        Material material;
    };

vector<string> split (const string &s, const char delim){
    vector<string> result;
    stringstream ss (s);
    string item;

    while (getline (ss, item, delim)) {
        result.push_back (item);
    }
    return result;
}

template <class T>
inline const T & getElement(const vector<T> &elements, string &stringIndex){
    int index = std::stoi(stringIndex);
    if (index < 0){
        index = int(elements.size()) + index;
    }
    else{
        index -= 1;
    }
    return elements[index];
}

float angleBetweenV3(const Vector3f a, const Vector3f b)        {
    float angle = a.dot(b);
    angle /= (a.norm() * b.norm());
    return angle = acosf(angle);
}

bool SameSide(Vector3f p1, Vector3f p2, Vector3f a, Vector3f b){
    Vector3f cp1 = (b - a).cross((p1 - a));
    Vector3f cp2 = (b - a).cross((p2 - a));

    if (cp1.dot(cp2) >= 0)
        return true;
    else
        return false;
}

bool inTriangle(Vector3f point, Vector3f tri1, Vector3f tri2, Vector3f tri3){
    // Test to see if it is within an infinite prism that the triangle outlines.
    bool within_tri_prisim = SameSide(point, tri1, tri2, tri3) && SameSide(point, tri2, tri1, tri3)
                                && SameSide(point, tri3, tri1, tri2);

    // If it isn't it will never be on the triangle
    if (!within_tri_prisim)
        return false;

    // Calulate Triangle's Normal
    Vector3f u = tri2 - tri1;
    Vector3f v = tri3 - tri1;
    Vector3f normal = u.cross(v);

    // Project the point onto this normal
    Vector3f bn = normal.normalized();
    Vector3f proj = bn * point.dot(bn);;

    // If the distance from the triangle to the point is 0
    //	it lies on the triangle
    if (proj.norm() == 0)
        return true;
    else
        return false;
}

// Generate vertices from a list of positions,
//	tcoords, normals and a face line
void genVerticesFromRawOBJ(vector<Vertex> &vertices, const vector<Vector3f> &positions, const vector<Vector2f> &tCoords, 
        const vector<Vector3f> &normals, vector<string> face){
    vector<string> stringVertex;
    Vertex vertex;
    bool noNormal = false;

    // For every given vertex do this
    for (int i = 0; i < int(face.size()); i++){
        // See What type the vertex is.
        int vtype;
        char delim = '/';
        stringVertex = split(face[i], delim);

        // Check for just position - v1
        if (stringVertex.size() == 1){
            // Only position
            vtype = 1;
        }

        // Check for position & texture - v1/vt1
        else if (stringVertex.size() == 2){
            // Position & Texture
            vtype = 2;
        }

        // Check for Position, Texture and Normal - v1/vt1/vn1
        // or if Position and Normal - v1//vn1
        else if (stringVertex.size() == 3){
            if (stringVertex[1] != "")
            {
                // Position, Texture, and Normal
                vtype = 4;
            }
            else
            {
                // Position & Normal
                vtype = 3;
            }
        }

        // Calculate and store the vertex
        switch (vtype){
            case 1: // P
            {
                vertex.position = getElement(positions, stringVertex[0]);
                vertex.textureCoordinates = Vector2f(0, 0);
                noNormal = true;
                vertices.push_back(vertex);
                break;
            }
            case 2: // P/T
            {
                vertex.position = getElement(positions, stringVertex[0]);
                vertex.textureCoordinates = getElement(tCoords, stringVertex[1]);
                noNormal = true;
                vertices.push_back(vertex);
                break;
            }
            case 3: // P//N
            {
                vertex.position = getElement(positions, stringVertex[0]);
                vertex.textureCoordinates = Vector2f(0, 0);
                vertex.normal = getElement(normals, stringVertex[2]);
                vertices.push_back(vertex);
                break;
            }
            case 4: // P/T/N
            {
                vertex.position = getElement(positions, stringVertex[0]);
                vertex.textureCoordinates = getElement(tCoords, stringVertex[1]);
                vertex.normal = getElement(normals, stringVertex[2]);
                vertices.push_back(vertex);
                break;
            }
            default:
            {
                break;
            }
        }
    }

    // take care of missing normals
    // these may not be truly acurate but it is the
    // best they get for not compiling a mesh with normals
    if (noNormal){
        Vector3f a = vertices[0].position - vertices[1].position;
        Vector3f b = vertices[2].position - vertices[1].position;

        Vector3f normal = a.cross(b);

        for (int i = 0; i < vertices.size(); i++){
            vertices[i].normal = normal;
        }
    }
}

// Triangulate a list of vertices into a face by printing
//	inducies corresponding with triangles within it
void vertexTriangluation(vector<unsigned int>& indices, vector<Vertex>& vertices){
    // If there are 2 or less verts,
    // no triangle can be created,
    // so exit
    if(vertices.size() < 3){
        return;
    }
    // If it is a triangle no need to calculate it
    if(vertices.size() == 3){
        indices.push_back(0);
        indices.push_back(1);
        indices.push_back(2);
        return;
    }

    while(true){
        // For every vertex
        for(int i = 0; i < int(vertices.size()); i++){
            // pPrev = the previous vertex in the list
            Vertex pPrev;
            if(i == 0){
                pPrev = vertices[vertices.size() - 1];
            }
            else{
                pPrev = vertices[i - 1];
            }

            // pCur = the current vertex;
            Vertex pCur = vertices[i];

            // pNext = the next vertex in the list
            Vertex pNext;
            if(i == vertices.size() - 1){
                pNext = vertices[0];
            }
            else{
                pNext = vertices[i + 1];
            }

            // Check to see if there are only 3 verts left
            // if so this is the last triangle
            if(vertices.size() == 3){
                // Create a triangle from pCur, pPrev, pNext
                for (int j = 0; j < int(vertices.size()); j++){
                    if (vertices[j].position == pCur.position){
                        indices.push_back(j);
                    }
                    if(vertices[j].position == pPrev.position){
                        indices.push_back(j);

                    }
                    if(vertices[j].position == pNext.position){
                        indices.push_back(j);
                    }
                }
                break;
            }
            if (vertices.size() == 4){
                // Create a triangle from pCur, pPrev, pNext
                for (int j = 0; j < int(vertices.size()); j++)
                {
                    if (vertices[j].position == pCur.position){
                        indices.push_back(j);
                    }
                    if(vertices[j].position == pPrev.position){
                        indices.push_back(j);

                    }
                    if(vertices[j].position == pNext.position){
                        indices.push_back(j);
                    }
                }

                Vector3f tempVec;
                for (int j = 0; j < int(vertices.size()); j++){
                    if (vertices[j].position != pCur.position && vertices[j].position != pPrev.position && vertices[j].position != pNext.position){
                        tempVec = vertices[j].position;
                        break;
                    }
                }

                // Create a triangle from pCur, pPrev, pNext
                for (int j = 0; j < int(vertices.size()); j++){
                    if (vertices[j].position == pPrev.position){
                        indices.push_back(j);
                    }
                    if (vertices[j].position == pNext.position){
                        indices.push_back(j);
                    }
                    if (vertices[j].position == tempVec){
                        indices.push_back(j);
                    }
                }

                break;
            }

            // If Vertex is not an interior vertex
            float angle = angleBetweenV3(pPrev.position - pCur.position, pNext.position - pCur.position) * (180 / 3.14159265359);
            if (angle <= 0 && angle >= 180)
                continue;

            // If any vertices are within this triangle
            bool inTri = false;
            for (int j = 0; j < int(vertices.size()); j++){
                bool temp = inTriangle(vertices[j].position, pPrev.position, pCur.position, pNext.position);
                if (temp && vertices[j].position != pPrev.position && vertices[j].position != pCur.position && vertices[j].position != pNext.position){
                    inTri = true;
                    break;
                }
            }
            if (inTri){
                continue;
            }

            // Create a triangle from pCur, pPrev, pNext
            for (int j = 0; j < int(vertices.size()); j++){
                if (vertices[j].position == pCur.position){
                    indices.push_back(j);
                }
                if (vertices[j].position == pPrev.position){
                    indices.push_back(j);
                }
                if (vertices[j].position == pNext.position){
                    indices.push_back(j);
                }
            }

            // Delete pCur from the list
            for (int j = 0; j < int(vertices.size()); j++){
                if (vertices[j].position == pCur.position){
                    vertices.erase(vertices.begin() + j);
                    break;
                }
            }

            // reset i to the start
            // -1 since loop will add 1 to it
            i = -1;
        }

        // if no triangles were created
        if (indices.size() == 0)
            break;

        // if no more vertices
        if (vertices.size() == 0)
            break;
    }
}

Material LoadMaterial(string path, string fileName){
    if(fileName.substr(fileName.size() - 4, 4) != ".mtl"){
        fileName.append(".mtl");
    }
    std::ifstream file(path + fileName);
    if(!file.is_open()){
        string badFile = "can't open file ";
        badFile.append(path);
        throw badFile; 
    }

    Material material;
    // Go through each line looking for material variables
    string currentLine;
    while (getline(file, currentLine)){
        // new material and material name
        istringstream iss(currentLine);
        string prefix;
        iss >> prefix;
        // Ambient Color
        if (prefix == "Ka"){
            Vector3f light;
            iss >> light[0] >> light[1] >> light[2];
            material.ka = light;
        }
        // Diffuse Color
        if (prefix == "Kd"){
            Vector3f light;
            iss >> light[0] >> light[1] >> light[2];
            material.kd = light;
        }
        // Specular Color
        if (prefix == "Ks"){
            Vector3f light;
            iss >> light[0] >> light[1] >> light[2];
            material.ks = light;
        }
        // Specular Exponent
        if (prefix == "Ns"){
            float ns;
            iss >> ns;
            material.shininessExponant = ns;
        }
        // Optical Density
        if (prefix == "Ni"){
            float ni;
            iss >> ni;
            material.lightAbsorption = ni;
        }
        // Dissolve
        if (prefix == "d"){
            //don't know what dissolve is or if we need it/
        }
        // Illumination
        if (prefix == "illum"){
            float illum;
            iss >> illum;
            material.lightEmission = illum;
        }
        // Ambient Texture Map
        if (prefix == "map_Ka"){
            string map;
            iss >> map;
            map = path + map; 
            material.ambiantTextureFile = map;
        }
        // Diffuse Texture Map
        if (prefix == "map_Kd"){
            string map;
            iss >> map;
            map = path + map; 
            material.diffuseTextureFile = map;
        }
        // Specular Texture Map
        if (prefix == "map_Ks"){
            string map;
            iss >> map;
            map = path + map; 
            material.specularTextureFile = map;
        }
        // Specular Hightlight Map
        if (prefix == "map_Ns"){
            string map;
            iss >> map;
            map = path + map; 
            material.specularShininessTextureFile = map;
        }
        // Alpha Texture Map
        if (prefix == "map_d"){
            string map;
            iss >> map;
            map = path + map; 
            material.opacityTextureFile = map;
        }
        // Bump Map
        if (prefix == "map_Bump" || prefix == "map_bump" || prefix == "bump")
        {
            string map;
            iss >> map;
            map = path + map; 
            material.bumpMapTextureFile = map;
        }
    }
    return material;
}

Mesh loadFile(string path){
    // If the file is not an .obj file return false
    cerr << "entered loadFile\n";
    Mesh object;
    Material material;
    if(path.substr(path.size() - 4, 4) != ".obj"){
        string badFile = "file not .obj ";
        badFile.append(path);
        throw badFile; 
    }

    std::ifstream file(path);

    if(!file.is_open()){
        string badFile = "can't open file ";
        badFile.append(path);
        throw badFile; 
    }

    vector<Vector3f> positions;
    vector<Vector2f> tCoords;
    vector<Vector3f> normals;
    vector<Vertex> vertices;
    vector<unsigned int> indices;
    vector<std::string> meshMatNames;
    Mesh tempMesh;
    string currentLine;
    int lineNum = 0;
    while(getline(file, currentLine)){
        istringstream iss(currentLine);
        string prefix;
        iss >> prefix;
        cerr << "\ron line " << ++lineNum << " a " << prefix;
        // Generate a Vertex Position
        if(prefix == "v"){
            Vector3f vertex;
            iss >> vertex[0] >> vertex[1] >> vertex[2];
            positions.push_back(vertex);
        }
        // Generate a Vertex Texture Coordinate
        else if(prefix == "vt"){
            Vector2f vertex;
            iss >> vertex[0] >> vertex[1];;
            tCoords.push_back(vertex);
        }
        // Generate a Vertex Normal;
        else if(prefix == "vn"){
            Vector3f vertex;
            iss >> vertex[0] >> vertex[1] >> vertex[2];

            normals.push_back(vertex);
        }
        // Generate a Face (vertices & indices)
        else if(prefix == "f"){
            // Generate the vertices
            std::vector<Vertex> verts;
            vector<string> face;
            for(string vertData; iss >> vertData;){
                face.push_back(vertData);
            }
            genVerticesFromRawOBJ(verts, positions, tCoords, normals, face);

            // Add Vertices
            for(int i = 0; i < int(verts.size()); i++){
                vertices.push_back(verts[i]);
            }

            std::vector<unsigned int> iIndices;

            vertexTriangluation(iIndices, verts);

            // Add Indices
            for(int i = 0; i < int(iIndices.size()); i++){
                unsigned int indnum = (unsigned int)((vertices.size()) - verts.size()) + iIndices[i];
                indices.push_back(indnum);
            }
        }
        else if(prefix == "usemtl"){
            string matFile;
            iss >> matFile;
            string directory;
            const size_t lastSlashIndex = path.rfind('/');
            if (std::string::npos != lastSlashIndex)
            {
                directory = path.substr(0, lastSlashIndex);
            }
            directory += "/";
            material = LoadMaterial(directory, matFile);
        }
    }
    cout << "\n";
    if(!indices.empty() && !vertices.empty()){
        // Create Mesh
        object = Mesh(vertices, indices);
        object.material = material;
    }

    file.close();
    return object;
}

Object meshToHalfEdge(const Mesh& mesh) {
    Object object;
    object.material = mesh.material;
    // Map to keep track of edges for setting up twins later
    // Key: pair of vertex indices (smaller index first), Value: half-edge pointer
    std::map<std::pair<int, int>, std::shared_ptr<HalfEdge>> edgeMap;
    
    int vertexIdCounter = 0;
    int halfEdgeIdCounter = 0;
    int faceIdCounter = 0;
    
    // Create vertices from mesh
    std::vector<std::shared_ptr<Vertex>> meshVertices;
    meshVertices.reserve(mesh.Vertices.size());
    
    // First pass: create all vertices
    for (const auto& objVert : mesh.Vertices) {
        auto vertex = std::make_shared<Vertex>(vertexIdCounter++);
        vertex->position = Vector3f(objVert.position.x(), objVert.position.y(), objVert.position.z());
        vertex->normal = Vector3f(objVert.normal.x(), objVert.normal.y(), objVert.normal.z());
        vertex->textureCoordinates = Vector2f(objVert.textureCoordinates.x(), objVert.textureCoordinates.y());
        
        // We'll set the halfEdge pointer later
        meshVertices.push_back(vertex);
        object.vertices.push_back(vertex);
    }
    
    // Process faces (triangles in the mesh)
    for (size_t i = 0; i < mesh.Indices.size(); i += 3) {
        auto face = std::make_shared<Face>(faceIdCounter++);
        object.faces.push_back(face);
                    
        // Create three half-edges for this triangle
        std::shared_ptr<HalfEdge> he[3];
        for (int j = 0; j < 3; j++) {
            he[j] = std::make_shared<HalfEdge>(halfEdgeIdCounter++);
            object.halfEdges.push_back(he[j]);
            
            // Set face for each half-edge
            he[j]->face = face;
        }
        
        // Connect the half-edges in a cycle and set up vertices
        for (int j = 0; j < 3; j++) {
            // Set the vertices for each half-edge
            int vertexIndex = mesh.Indices[i + j];
            he[j]->vertex = meshVertices[vertexIndex];
            
            // Set pointers for next and previous
            he[j]->next = he[(j + 1) % 3];
            he[j]->previous = he[(j + 2) % 3];
            
            // Update vertex's outgoing half-edge if not already set
            if (!he[j]->vertex->halfEdge.lock()) {
                he[j]->vertex->halfEdge = he[j];
            }
            
            // Store the half-edge for twin setup
            int nextIndex = mesh.Indices[i + (j + 1) % 3];
            
            // Always use smaller vertex id first to ensure consistent mapping
            int v1 = vertexIndex;
            int v2 = nextIndex;
            if (v1 > v2) std::swap(v1, v2);
            
            auto edgePair = std::make_pair(v1, v2);
            
            // If this edge already exists in our map, we've found twins
            auto it = edgeMap.find(edgePair);
            if (it != edgeMap.end()) {
                // We found the twin! Link them together
                std::shared_ptr<HalfEdge> twin = it->second;
                
                // Determine if they actually are twins (direction is opposite)
                if (twin->vertex->id != nextIndex) {
                    // Set the twin relationships
                    he[j]->twin = twin;
                    twin->twin = he[j];
                }
            } else {
                // Store this half-edge for future twin finding
                edgeMap[edgePair] = he[j];
            }
        }
        
        // Set face's half-edge pointer to one of its half-edges
        face->halfEdge = he[0];
        
        // Calculate face normal
        Vector3f v0 = he[0]->vertex->position;
        Vector3f v1 = he[1]->vertex->position;
        Vector3f v2 = he[2]->vertex->position;
        Vector3f e1 = v1 - v0;
        Vector3f e2 = v2 - v0;
        face->normal = e1.cross(e2).normalized();
    }
    
    // Compute vertex normals
    for (auto& vertex : object.vertices) {
        Vector3f normal = Vector3f::Zero();
        auto half_edges = vertex->neighbourHalfEdges();
        
        for (const auto& he : half_edges) {
            if (he->face) {
                normal += he->face->normal;
            }
        }
        
        if (normal != Vector3f::Zero()) {
            vertex->normal = normal.normalized();
        } 
    }
    return object;
}

Object load(string fileLocation) {
    Object object;
    try{
        Mesh obj = loadFile(fileLocation);
        std::cout << "OBJ file loaded successfully: " << fileLocation << std::endl;
        object = meshToHalfEdge(obj);
    }
    catch(string message){
        std::cerr << "\nFailed to load OBJ file: " << fileLocation << ", reason: " << message << std::endl;
        return object;
    }
    if(!objectFacesConsistent){
        makeObjectFacesConsistent(object);
    }
    if(!objectTri(object)){
        makeObjectTri(object);
    }
    if(!objectClosed){
        cerr << fileLocation << " not closed\n";
    }
    if(!objectConnected){
        cerr << fileLocation << " not fully connected\n";
    }
    if(!objectManifold){
        cerr << fileLocation << " not manifold\n";
    }
    return object;
}