#include <memory>
#include <vector>
#include <Eigen/Dense>
class Object;
class Face;
class HalfEdge;
class Vertex;

class Object{
    public:
        std::vector<Face> faces;
        std::vector<HalfEdge> halfEdges;
        std::vector<Vertex> vertices;
        std::string textureFile;
        Object(){}
};

class Face{
    public:
        int id;
        std::shared_ptr<HalfEdge> halfEdge;
        Eigen::Vector3f normal;
        //need to figure out how to represent material
        //auto material;
        Face(){}
        Face(int _id): id(_id){}
        std::vector<std::shared_ptr<HalfEdge>> getHalfEdges(){
            std::vector<std::shared_ptr<HalfEdge>> halfEdges;
            std::shared_ptr<HalfEdge> currentHalfEdge = halfEdge;
            do{
                halfEdges.push_back(currentHalfEdge);
                currentHalfEdge = halfEdge->next;
            }
            while(currentHalfEdge != halfEdge);
        }
        std::vector<std::shared_ptr<Vertex>> getVertices(){

        }
};

class HalfEdge{
    public:
        int id;
        std::shared_ptr<HalfEdge> next;
        std::shared_ptr<HalfEdge> previous; //make doubly linked, as being singly linked is stupid
        std::shared_ptr<HalfEdge> twin;
        std::shared_ptr<Vertex> vertex; //origin vertex
        std::shared_ptr<Face> face;
        HalfEdge(int _id): id(_id){}
};

class Vertex{
    public:
        int id;
        std::shared_ptr<HalfEdge> halfEdge;
        Eigen::Vector3f position;
        Eigen::Vector3f textureCoordinates;
        Eigen::Vector3f colour; //if not using texture, in range [0, 255]
        Vertex(int _id): id(_id){}
        std::vector<std::shared_ptr<Vertex>> getNeighbourVertices(){
            std::vector<std::shared_ptr<Vertex>> neighbourhood;
            if(halfEdge == nullptr){
                return neighbourhood;
            }
            std::shared_ptr<HalfEdge> currentHalfEdge = halfEdge;
            do {
                if(currentHalfEdge == nullptr || currentHalfEdge->twin == nullptr || currentHalfEdge->twin->vertex == nullptr){
                    break;
                }
                neighbourhood.push_back(currentHalfEdge->twin->vertex);
                currentHalfEdge = currentHalfEdge->twin->next;
            }
            while(currentHalfEdge != halfEdge);
            return neighbourhood; 
        }
};