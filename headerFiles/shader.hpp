#pragma once

#include <Eigen/Eigen>
#include <vector>
#include "texture.hpp"
#include "light.hpp"
#include <Eigen/Core>

// Fragment shader payload structure from CLab2
struct fragment_shader_payload
{
    fragment_shader_payload(const Eigen::Vector3f& col, const Eigen::Vector3f& nor, const Eigen::Vector2f& tc, const std::vector<Light> vl, Texture* tex) :
        colour(col), normal(nor), tex_coords(tc), view_lights(vl), texture(tex) {}

    Eigen::Vector3f view_pos;
    Eigen::Vector3f colour;
    Eigen::Vector3f normal;
    Eigen::Vector2f tex_coords;
    std::vector<Light> view_lights;
    Texture* texture;
};

/*
struct Material {
    Eigen::Vector3f ambient;
    Eigen::Vector3f diffuse;
    Eigen::Vector3f specular;
    float shininess;

    Material()
        : ambient(0.1f, 0.1f, 0.1f),
          diffuse(0.6f, 0.6f, 0.6f),
          specular(0.3f, 0.3f, 0.3f),
          shininess(32.0f) {}
};*/