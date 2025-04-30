#include <include/half_edge.hpp>
#include <Eigen/Dense>

/*
#################################################################################
#                       Vertex-related Helper Functions                         #
#################################################################################
*/

// Iterate through all neighbour vertices around the vertex
std::vector<std::shared_ptr<Vertex>> Vertex::neighbor_vertices() {
    std::vector<std::shared_ptr<Vertex>> neighborhood;
    auto he = this->he;
    do {
        neighborhood.push_back(he->twin->vertex);
        he = he->twin->next;
    }
    while(he != this->he);
    return neighborhood; 
}


// Iterate through all half edges pointing away from the vertex
std::vector<std::shared_ptr<HalfEdge>> Vertex::neighbor_half_edges() {
    std::vector<std::shared_ptr<HalfEdge>> neighborhood;
    auto he = this->he;
    do {
        neighborhood.push_back(he);
        he = he->twin->next;
    }
    while(he != this->he);
    return neighborhood;
}


/*
#################################################################################
#                         Face-related Helper Functions                         #
#################################################################################
*/

// Iterate through all member vertices of the face
std::vector<std::shared_ptr<Vertex>> Face::vertices() {
    std::vector<std::shared_ptr<Vertex>> member_vertices;
    auto he = this->he;
    do {
        member_vertices.push_back(he->vertex);
        he = he->next;
    }
    while(he != this->he);
    return member_vertices;
}


// TODO(Task1-3): implement this function to compute the area of the triangular face 
float Face::get_area(){
    float area = 0;
    // get the vertices of the face
    auto vertices = Face::vertices();
    Eigen::Vector3f v0 = vertices[0]->pos, v1 = vertices[1]->pos, v2 = vertices[2]->pos;
    // --- Start your code here --- //
    //area of triangle in 3d space
    //will use formula area = magnitude(AB cross AC) / 2
    Eigen::Vector3f ab = v0 - v1;
    Eigen::Vector3f ac = v0 - v2;
    Eigen::Vector3f cross = ab.cross(ac);
    float magnatude = sqrt(pow(cross.x(), 2) + pow(cross.y(), 2) + pow(cross.z(), 2));
    area = magnatude / 2;
    // --- End your code here --- //
    return area;
}

// TODO:(Task1-4) implement this function to compute the signed volume of the triangular face 
// reference: http://chenlab.ece.cornell.edu/Publication/Cha/icip01_Cha.pdf eq.(5)
float Face::get_signed_volume(){
    float volume = 0 ;
    // get the vertices of the face
    auto vertices = Face::vertices();
    Eigen::Vector3f v0 = vertices[0]->pos, v1 = vertices[1]->pos, v2 = vertices[2]->pos;
    // --- Start your code here --- //
    //formula: Vi = 1/6 (-x3y2z1 + x2y3z1 + x3y1z2 - x1y3z2 - x2y1z3 + x1y2z3)
    float arg1 = -(v2.x() * v1.y() * v0.z());
    float arg2 = (v1.x() * v2.y() * v0.z());
    float arg3 = (v2.x() * v0.y() * v1.z());
    float arg4 = -(v0.x() * v2.y() * v1.z());
    float arg5 = -(v1.x() * v0.y() * v2.z());
    float arg6 = (v0.x() * v1.y() * v2.z());
    volume = (arg1 + arg2 + arg3 + arg4 + arg5 + arg6) / 6.0;
    // --- End your code here --- //
    return volume;
}

// TODO:(Task1-5) Use the half-edge data structure, compute the surface normal vector for each triangle face. 
void Face::compute_normal(){
    /*
        Hint: the comment below is a reference for you to implement the function in a mathematical way.
        You should rewrite it in a half-edge data structure way.
        for example:
            std::shared_ptr<HalfEdge> he0 = this->he;
            Eigen::Vector3f v0 = he0->vertex->pos;
            ...     
    */
    // --- Start your code here --- //
    std::vector<std::shared_ptr<Vertex>> vertices = Face::vertices();
    Eigen::Vector3f a = vertices[0]->pos;
    Eigen::Vector3f b = vertices[1]->pos;
    Eigen::Vector3f c = vertices[2]->pos;
    Eigen::Vector3f ab = a - b;
    Eigen::Vector3f ac = a - c;
    this->normal = ab.cross(ac);
    // --- End your code here --- //
}

