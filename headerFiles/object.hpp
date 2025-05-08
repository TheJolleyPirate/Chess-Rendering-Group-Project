#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>
class Object;
class Face;
class HalfEdge;
class Vertex;

class HalfEdge{
    public:
        int id; //unique within object
        std::shared_ptr<HalfEdge> next;
        std::weak_ptr<HalfEdge> previous; //make doubly linked, as being singly linked is stupid
        std::weak_ptr<HalfEdge> twin;
        std::shared_ptr<Vertex> vertex; //origin vertex
        std::shared_ptr<Face> face;
        HalfEdge(int _id): id(_id){}
};

class Face{
    public:
        int id; //unique within object
        std::weak_ptr<HalfEdge> halfEdge;
        Eigen::Vector3f normal;
        Eigen::Vector3f colour; //for if object does not have a texture
        //need to figure out how to represent material
        //auto material;
        Face(){}
        Face(int _id): id(_id){}
        std::vector<std::shared_ptr<HalfEdge>> getHalfEdges(){
            std::vector<std::shared_ptr<HalfEdge>> halfEdges;
            std::shared_ptr<HalfEdge> currentHalfEdge = halfEdge.lock();
            int halfEdgeId = currentHalfEdge->id;
            do{
                halfEdges.push_back(currentHalfEdge);
                currentHalfEdge = currentHalfEdge->next;
            }
            while(currentHalfEdge->id != halfEdgeId);
            return halfEdges;
        }
        std::vector<std::shared_ptr<Vertex>> getVertices(){
            std::vector<std::shared_ptr<Vertex>> vertices;
            std::shared_ptr<HalfEdge> currentHalfEdge = halfEdge.lock();
            int halfEdgeId = currentHalfEdge->id;
            do{
                vertices.push_back(currentHalfEdge->vertex);
                currentHalfEdge = currentHalfEdge->next;
            }
            while(currentHalfEdge->id != halfEdgeId);
            return vertices;
        }
};

class Vertex{
    public:
        int id; //unique within object
        std::weak_ptr<HalfEdge> halfEdge;
        Eigen::Vector3f position;
        Eigen::Vector3f normal;
        Eigen::Vector3f textureCoordinates;
        Eigen::Vector3f colour; //if not using texture, in range [0, 255]
        Vertex(int _id): id(_id){}
        std::vector<std::shared_ptr<Vertex>> getNeighbourVertices(){
            std::vector<std::shared_ptr<Vertex>> neighbourhood;
            std::shared_ptr<HalfEdge> currentHalfEdge = halfEdge.lock();
            int halfEdgeId = currentHalfEdge->id;
            if(!currentHalfEdge){
                return neighbourhood;
            }
            do {
                std::shared_ptr<HalfEdge> twin = currentHalfEdge->twin.lock();
                if(!currentHalfEdge || !twin || !twin->vertex){
                    break;
                }
                neighbourhood.push_back(twin->vertex);
                currentHalfEdge = twin->next;
            }
            while(currentHalfEdge->id != halfEdgeId);
            return neighbourhood; 
        }

        // Get all the half edges that originate from this vertex
        std::vector<std::shared_ptr<HalfEdge>> neighbourHalfEdges() {
            std::vector<std::shared_ptr<HalfEdge>> neighbourhood;
            auto he = this->halfEdge;
            do {
                neighbourhood.push_back(he.lock());
                he = he.lock()->twin.lock()->next;
            }
            while(he.lock() != this->halfEdge.lock());
            return neighbourhood;
        }

        // Compute the normal vector for this vertex by averaging the normals of the faces it belongs to
        Eigen::Vector3f computeNormal() {
            Eigen::Vector3f normal = Eigen::Vector3f::Zero();
            auto halfEdges = neighbourHalfEdges();
            for (auto& he : halfEdges) {
                if (he->face == nullptr) {
                    continue;
                }
                normal += he->face->normal;
            }
            this->normal = normal.normalized();
            return this->normal;
        }
};

class Object{
    public:
        std::vector<std::shared_ptr<Face>> faces;
        std::vector<std::shared_ptr<HalfEdge>> halfEdges;
        std::vector<std::shared_ptr<Vertex>> vertices;
};
