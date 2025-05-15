#pragma once

#include <memory>
#include <vector>
#include <Eigen/Dense>
class Object;
class Material;
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
        Eigen::Vector2f textureCoordinates;
        Eigen::Vector3f colour; //if you are doing colour per vertex
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

class Material{
    public:
        Eigen::Vector3f colour;
        std::string textureFile;
        float opacity;
        float ior; // index of refraction
        float shininessExponant;
        float lightEmission;
        bool textured;
        float kd; //diffuse light
        float ks; //specular light
        float ka; //ambiant light

        Material(){
            kd = 0.8;
            ks = 0.2;
            ka = kd;
            shininessExponant = 25;
            lightEmission = 0;
            textured=false;
            ior=2;
        }

        Material(Eigen::Vector3f _colour){
            colour = _colour;
            kd = 0.8;
            ks = 0.2;
            ka = kd;
            shininessExponant = 25;
            lightEmission = 0;
            textured=false;
            ior=2;
        }

        Material(std::string _textureFile){
            kd = 0.8;
            ks = 0.2;
            ka = kd;
            shininessExponant = 25;
            lightEmission = 0;
            textured=true;
            textureFile = _textureFile;
            ior=2;
        }

        float getKR(const Eigen::Vector3f &incidentPoint, const Eigen::Vector3f &normal){
            float cosi = std::clamp(-1.0f, 1.0f, incidentPoint.dot(normal));
            float etai = 1, etat = ior;
            float kr;
            if (cosi > 0) {  std::swap(etai, etat); }
            // Compute sini using Snell's law
            float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
            // Total internal reflection
            if (sint >= 1) {
                kr = 1;
            }
            else {
                float cost = sqrtf(std::max(0.f, 1 - sint * sint));
                cosi = fabsf(cosi);
                float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
                float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
                kr = (Rs * Rs + Rp * Rp) / 2;
            }
            // As a consequence of the conservation of energy, transmittance is given by:
            // kt = 1 - kr;
            return kr;
        }
};

class Object{
    public:
        std::vector<std::shared_ptr<Face>> faces;
        std::vector<std::shared_ptr<HalfEdge>> halfEdges;
        std::vector<std::shared_ptr<Vertex>> vertices;
        Material material;
};
