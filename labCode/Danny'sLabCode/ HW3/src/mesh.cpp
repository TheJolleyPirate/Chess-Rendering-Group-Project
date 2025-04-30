#include <include/mesh.hpp>
#include <include/meshChecker.hpp>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <queue>
#include <algorithm>
#include <assert.h>
#include <unordered_set> // for is_single_component


bool Mesh::load_obj(const std::string& filepath) {
    if (this->display_vertices.size() > 0) {
        this->display_vertices.clear();
        this->display_faces.clear();
        this->display_normals.clear();
    }
    if (filepath.substr(filepath.size() - 4, 4) != ".obj") {
        std::cerr << "Only obj file is supported." << std::endl;
        return false;
    }
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filepath << std::endl;
        return false;
    }
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;
        if ("v" == prefix) {
            Vector3f vertex;
            iss >> vertex[0] >> vertex[1] >> vertex[2];
            this->display_vertices.push_back(vertex);
        } else if ("f" == prefix) {
            Vector3i face;
            iss >> face[0] >> face[1] >> face[2];
            face = face.array() - 1;
            this->display_faces.push_back(face);
        }
    }
    return true;
}


/*
    TODO:(Task1-1)
        Implement this function to convert a loaded OBJ format mesh into half-edge-based mesh. 
    HINT:
        The loaded OBJ mesh information is stored in "this->display_vertices" and "this->display_faces" class variables,
        and the half-edge-based mesh information is stored in "this->vertices", "this->faces", "this->half_edges", "this->edges".
        You'll need to find a way to construct these class members and setting up associated attributes properly.
        The id for each type of objects starts from 0;
*/
void Mesh::convert_obj_format_to_mesh() {
    if (this->vertices.size() > 0) {
        this->vertices.clear();
        this->faces.clear();
        this->half_edges.clear();
        this->edges.clear();
    }
    // --- Start your code here --- //
    using namespace std;
    using namespace Eigen;
    vector<shared_ptr<Vertex>> vertices(this->display_vertices.size(), nullptr);
    vector<shared_ptr<Face>> faces;
    vector<shared_ptr<HalfEdge>> halfEdges;
    vector<shared_ptr<Edge>> edges;
    int faceIdent, heIdent, edgeIdent;
    faceIdent = heIdent = edgeIdent = 0;
    vector<vector<shared_ptr<HalfEdge>>> heDest;
    heDest.resize(this->display_vertices.size());
    for(Eigen::Vector3i faceCoords : this->display_faces){
        shared_ptr<Face> face = make_shared<Face>(Face(faceIdent++));
        //set face colour
        face->color = {255, 255, 255}; 
        shared_ptr<HalfEdge> previous = nullptr;
        shared_ptr<Vertex> firstVert;
        shared_ptr<HalfEdge> firstHE;
        for(int vertNum: faceCoords){
            shared_ptr<HalfEdge> he = make_shared<HalfEdge>(HalfEdge(heIdent++));
            if(previous != nullptr){
                previous->next = he;
                heDest[vertNum].push_back(previous);
            }
            else{
                firstHE = he;
            }
            shared_ptr<Vertex> v;
            if(vertices[vertNum] == nullptr){
                v = make_shared<Vertex>(Vertex(vertNum));
                v->pos = this->display_vertices[vertNum];
                v->he = he;
                vertices[vertNum] = v;
            }
            else{
                v = vertices[vertNum];
            }
            he->vertex = v;
            he->face = face;
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
                if(twin->vertex->id == dest){
                    if(!twin->has_twin()){
                        edges.push_back(make_shared<Edge>(Edge(he, edgeIdent++)));
                        he->twin = twin;
                        twin->twin = he;
                        break;
                    }
                }
            }
        }
    }
    for(int i = 0; i < vertices.size(); ++i){
        if(vertices[i] == nullptr){
            vertices[i] = make_shared<Vertex>(i);
            vertices[i]->pos = this->display_vertices[i];
        }
    }
    this->vertices = vertices;
    this->faces = faces;
    this->half_edges = halfEdges;
    this->edges = edges;
    // --- End your code here --- //
    std::cout << "====== Mesh Information ======" << std::endl;
    this->print_mesh_info();
}

// TODO:(Task1-2) Implement this function to compute the genus number 
int Mesh::compute_genus() {
    int genus = 0;
    // --- Start your code here --- //
    //using Euler-Poincaré formula: f - e + v = 2 - 2g
    //we get that g = (e - v - f + 2) / 2
    int v = this->vertices.size();
    int e = this->edges.size();
    int f = this->faces.size();
    genus = (e - v - f + 2) / 2;
    // --- End your code here --- //
    return genus;
}

// TODO:(Task1-3) Implement this function to compute the surface area of the mesh 
// HINT: You can first implement Face::get_area() to compute the surface area of each face, and then sum them up 
float Mesh::compute_surface_area() {
    float total_surface_area = 0;
    // --- Start your code here --- //
    for(std::shared_ptr<Face> f : this->faces){
        total_surface_area += f->get_area();
    }
    // --- End your code here --- //
    return total_surface_area;
}

// TODO:(Task1-4) Implement this function to compute the volume of the mesh 
// HINT: You can first implement Face::get_signed_volume() to compute the volume associate with each face, and then sum them up 
float Mesh::compute_volume() {
    float total_volume = 0;
    // --- Start your code here --- //
    for(std::shared_ptr<Face> f : this->faces){
        total_volume += f->get_signed_volume();
    }
    total_volume = abs(total_volume);
    // --- End your code here --- //
    return total_volume;
}

// TODO(Task1-5): Use the half-edge data structure, compute the surface normal vector for each triangle face. 
// Complete the ``compute_normal()'' function in Face class
void Mesh::compute_normal_per_face() {
    for (auto& face : this->faces) {
        face->compute_normal();
    }
}

// TODO(Task1-6): Use the half-edge data structure, compute the normal vector for vertex. 
void Mesh::compute_normal_per_vertex() {
    // --- Start your code here --- //
    // replace this dummy code with your implementation
    for (auto& vertex : this->vertices) {
        vertex->he->face->compute_normal();
        vertex->normal = vertex->he->face->normal;
    }
    // --- End your code here --- //
}

// This function is used to convert the half-edge-based mesh back to OBJ format for checking 
void Mesh::convert_mesh_to_obj_format() {
    if (this->display_vertices.size() > 0) {
        this->display_vertices.clear();
        this->display_faces.clear();
        this->display_normals.clear();
    }
    std::map<std::shared_ptr<Vertex>, int> indices;
    int temp_idx = 0;
    for (const auto& vertex : this->vertices) {
        indices[vertex] = temp_idx;
        temp_idx++;
        this->display_vertices.push_back(vertex->pos);
        this->display_normals.push_back(vertex->normal);
    }
    for (auto& face : this->faces) {
        Vector3i face_vert_id;
        int idx = 0;
        for (const auto& vertex : face->vertices()) {
            face_vert_id[idx] = indices[vertex];
            idx++;
        }
        this->display_faces.push_back(face_vert_id);
    }
}

bool Mesh::save_obj(const std::string& filepath) const {
    if (filepath.substr(filepath.size() - 4, 4) != ".obj") {
        std::cerr << "Only obj file is supported." << std::endl;
        return false;
    }
    std::ofstream out_file(filepath);
    if (!out_file.is_open()) {
        std::cerr << "Failed to open file: " << filepath << std::endl;
        return false;
    }
    // write vertices
    for (const auto &vertex : this->display_vertices) {
        out_file << "v " << vertex[0] << " " << vertex[1] << " " << vertex[2] << "\n";
    }
     // write normals
    for (const auto &normal : this->display_normals) {
        out_file << "vn " << normal[0] << " " << normal[1] << " " << normal[2] << "\n";
    }
    // write faces
    for (const auto &face : this->display_faces) {
        out_file << "f " << face[0] + 1 << " " << face[1] + 1 << " " << face[2] + 1 << "\n";
    }
    out_file.close();
  return true;
}

void Mesh::remove_invalid_components() {
    this->vertices.erase(
        std::remove_if(this->vertices.begin(), this->vertices.end(), [](std::shared_ptr<Vertex> v) { return !v->exists; }), 
        this->vertices.end()
    );
    this->faces.erase(
        std::remove_if(this->faces.begin(), this->faces.end(), [](std::shared_ptr<Face> f) { return !f->exists; }), 
        this->faces.end()
    );
    this->half_edges.erase(
        std::remove_if(this->half_edges.begin(), this->half_edges.end(), [](std::shared_ptr<HalfEdge> he) { return !he->exists; }), 
        this->half_edges.end()
    );
    this->edges.erase(
        std::remove_if(this->edges.begin(), this->edges.end(), [](std::shared_ptr<Edge> e) { return !e->exists; }), 
        this->edges.end()
    );
}

int Mesh::verify() {
    // Check Euler's formula
    assert(this->vertices.size() + this->faces.size() - this->edges.size() == 2);
    assert(this->edges.size() * 2 == this->half_edges.size());
    int rvalue = 0;
    for (auto& v : this->vertices) {
        if (v->exists) {
            if (!v->he->exists) {
                rvalue |= 1<<0;
            }
        }
    }
    for (auto& f : this->faces) {
        if (f->exists) {
            if (!f->he->exists) {
                rvalue |= 1<<1;
            }
            if (f->he->next->next->next != f->he) {
                rvalue |= 1<<2;
            }
        }
    }
    for (auto& he : this->half_edges) {
        if (he->exists) {
            if (!he->vertex->exists) {
                rvalue |= 1<<3;
            }
            if (!he->edge->exists) {
                rvalue |= 1<<4;
            }
            if (!he->face->exists) {
                rvalue |= 1<<5;
            }
            if (he->twin->twin != he) {
                rvalue |= 1<<6;
            }
        }
    }
    for (auto& e : this->edges) {
        if (e->exists) {
            if (!e->he->exists) {
                rvalue |= 1<<7;
            }
        }
    }
    
    return rvalue;
}

void Mesh::print_mesh_info() {
    std::cout << "number of faces: " << this->faces.size() << std::endl;
    std::cout << "number of vertices: " << this->vertices.size() << std::endl;
    std::cout << "number of half edges: " << this->half_edges.size() << std::endl;
    std::cout << "number of edges: " << this->edges.size() << std::endl;
}

// TODO:(task2) Implement the function to check the mesh properties
// Please feel free to add any helper functions if needed
// HINT: You can use the half-edge data structure to check the mesh properties but it's up to you.
std::string Mesh::check_mesh(const std::string& filepath){
    // mesh properties
    if (filepath.substr(filepath.size() - 4, 4) != ".obj") {
        std::cerr << "Only obj file is supported." << std::endl;
        return "";
    }
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filepath << std::endl;
        return "";
    }
    if (this->display_vertices.size() > 0) {
        this->display_vertices.clear();
        this->display_faces.clear();
        this->display_normals.clear();
    }
    if (this->vertices.size() > 0) {
        this->vertices.clear();
        this->faces.clear();
        this->half_edges.clear();
        this->edges.clear();
    }
    bool is_triangular = true;
    bool is_single_part = true;
    bool is_closed = true;
    bool is_manifold = true;
    bool is_consist_ori = true;
    float polygon_volume = 0; // for extension 4.2 only

    // --- Start your code here --- //
    // Write your code to load the mesh and replace the above dummy mesh properties by the real values
    std::map<std::string, float> meshInfo = checkMesh(filepath);
    is_triangular = meshInfo["is_triangular"];
    is_single_part = meshInfo["is_single_part"];
    is_closed = meshInfo["is_closed"];
    is_manifold = meshInfo["is_manifold"];
    is_consist_ori = meshInfo["is_consist_ori"];
    polygon_volume = meshInfo["polygon_volume"];
    
    // --- End your code here --- //
    // This output string will be used for the unit test with our own mesh files 
    // Y: Yes, N: No
    std::cout << "Checking mesh: " << filepath << std::endl;
    std::string output = "";
    output += (is_triangular ? "Y" : "N");
    output += (is_single_part ? "Y" : "N");
    output += (is_closed ? "Y" : "N");
    output += (is_manifold ? "Y" : "N");
    output += (is_consist_ori ? "Y" : "N");
    std::cout << output;
    if (!is_triangular && is_single_part && is_closed && is_manifold && is_consist_ori) {
        std::cout << "  polygon mesh model volume: " << polygon_volume << std::endl;
    }
    std::cout << std::endl;
    return output;
}