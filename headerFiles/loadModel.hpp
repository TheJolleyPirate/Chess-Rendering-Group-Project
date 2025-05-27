#pragma once

#include <object.hpp>

struct RawVertex{
    Eigen::Vector3f position;
    Eigen::Vector3f normal;
    Eigen::Vector2f textureCoords;
    int id;
    bool operator==(const RawVertex &other) const{
        return position == other.position && normal == other.normal && textureCoords == other.textureCoords;
    }
    bool operator<(const RawVertex &other) const{
        auto lhs = std::tie(
            position.x(), position.y(), position.z(),
            textureCoords.x(), textureCoords.y(),
            normal.x(),   normal.y(),   normal.z());
        auto rhs = std::tie(
            other.position.x(), other.position.y(), other.position.z(),
            other.textureCoords.x(), other.textureCoords.y(),
            other.normal.x(),   other.normal.y(),   other.normal.z());
        return lhs < rhs;
    }
};

struct Mesh{
    std::vector<RawVertex> vertices;
    std::vector<std::vector<int>> faces;
    Material material;
};

std::vector<std::string> split(const std::string &s, const char delim);
Mesh loadFile(std::string path);
Object meshToHalfEdge(const Mesh& mesh);
Object load(std::string fileLocation, std::string fileName);