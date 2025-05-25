#pragma once

#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>
#include "object.hpp"
#include "light.hpp"

class Scene {
public:
    std::vector<Object> objects;
    std::vector<Light> lights;

    Scene() = default;
    Scene(std::vector<Object> _objects, std::vector<Light> _lights)
        : objects(std::move(_objects)), lights(std::move(_lights)) {}

    void addObject(const Object& obj) { objects.push_back(obj); }
    void addLight(const Light& light) { lights.push_back(light); }

};

Scene loadSceneFromJson(std::map<std::string, Object> rawObjects, std::string fileJSON);
