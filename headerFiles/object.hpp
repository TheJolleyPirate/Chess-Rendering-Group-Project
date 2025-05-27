#pragma once

#include <memory>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include "texture.hpp"

class Object;
class Material;
class Face;
class HalfEdge;
class Vertex;

class HalfEdge{
    public:
        int id; //unique within face
        std::shared_ptr<HalfEdge> next;
        std::weak_ptr<HalfEdge> previous; //make doubly linked, as being singly linked is stupid
        std::weak_ptr<HalfEdge> twin;
        std::shared_ptr<Vertex> vertex; //origin vertex
        std::shared_ptr<Face> face;
        HalfEdge(int _id): id(_id){}
        HalfEdge(){}
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
        int id; //unique within face
        std::weak_ptr<HalfEdge> halfEdge;
        Eigen::Vector3f position;
        Eigen::Vector3f normal = Eigen::Vector3f(0, 0, 0);
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
        Vertex(){}

        // Get all the half edges that originate from this vertex
        //assumes that vertex is manifold
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

        Eigen::Vector3f computeNormal() {
            if(this->normal == Eigen::Vector3f(0, 0, 0)){
                this->normal = halfEdge.lock()->face->normal;
            }
            return this->normal;
        }

        bool operator < (const Vertex &other) const {
            return id < other.id;
        }
};

class Material{
    public:
        Eigen::Vector3f colour;
        std::string diffuseTextureFile;
        Texture diffuseTexture = Texture(""); //default empty texture;
        std::string mtlFile;
        //below are optional
        std::string specularTextureFile;
        std::string ambiantTextureFile;
        std::string specularShininessTextureFile;
        std::string opacityTextureFile; //alpha
        std::string bumpMapTextureFile;
        float opacity;
        float ior; // index of refraction
        float shininessExponant;// Specular Exponent
        float lightEmission;
        float lightAbsorption; //ocular density
        bool textured;
        Eigen::Vector3f kd; //diffuse light
        Eigen::Vector3f ks; //specular light
        Eigen::Vector3f ka; //ambiant light

        Material(){
            kd = {0.8, 0.8, 0.8};
            ks = {0.2, 0.2, 0.2};
            ka = kd;
            shininessExponant = 25;
            lightEmission = 0;
            opacity = 1;
            lightAbsorption = 0.1;
            textured = false;
            ior = 2;
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
        std::string objFile;
};
