#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <map>
#include "Eigen/Dense"

#include "headerFiles/meshChecker.hpp"
#include "headerFiles/object.hpp"
#include "labCode/Danny'sLabCode/HW2/OBJ_Loader.h"

using namespace std;
using namespace Eigen;

Object meshToHalfEdge(const objl::Loader& loader) {
    Object object;
    
    // Map to keep track of edges for setting up twins later
    // Key: pair of vertex indices (smaller index first), Value: half-edge pointer
    std::map<std::pair<int, int>, std::shared_ptr<HalfEdge>> edgeMap;
    
    int vertexIdCounter = 0;
    int halfEdgeIdCounter = 0;
    int faceIdCounter = 0;
    
    // Process each mesh in the loaded OBJ file
    for (const auto& mesh : loader.LoadedMeshes) {
        // Create vertices from mesh
        std::vector<std::shared_ptr<Vertex>> meshVertices;
        meshVertices.reserve(mesh.Vertices.size());
        
        // First pass: create all vertices
        for (const auto& objVert : mesh.Vertices) {
            auto vertex = std::make_shared<Vertex>(vertexIdCounter++);
            vertex->position = Vector3f(objVert.Position.X, objVert.Position.Y, objVert.Position.Z);
            vertex->normal = Vector3f(objVert.Normal.X, objVert.Normal.Y, objVert.Normal.Z);
            vertex->textureCoordinates = Vector3f(objVert.TextureCoordinate.X, objVert.TextureCoordinate.Y, 0.0f);
            
            // We'll set the halfEdge pointer later
            meshVertices.push_back(vertex);
            object.vertices.push_back(vertex);
        }
        
        // Process faces (triangles in the mesh)
        for (size_t i = 0; i < mesh.Indices.size(); i += 3) {
            auto face = std::make_shared<Face>(faceIdCounter++);
            object.faces.push_back(face);
            
            // Material color
            face->colour = Vector3f(mesh.MeshMaterial.Kd.X, mesh.MeshMaterial.Kd.Y, mesh.MeshMaterial.Kd.Z);
            
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
    objl::Loader loader;
    
    if (loader.LoadFile(fileLocation)) {
        std::cout << "OBJ file loaded successfully: " << fileLocation << std::endl;
        return meshToHalfEdge(loader);
    } else {
        std::cerr << "Failed to load OBJ file: " << fileLocation << std::endl;
        return Object();
    }
}