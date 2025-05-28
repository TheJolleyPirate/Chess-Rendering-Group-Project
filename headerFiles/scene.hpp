#pragma once

#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>
#include "object.hpp"
#include "light.hpp"

/*by Daniel Jolley-Rogers u7511912
defines a scene containing information about object, lights and where to put them*/
class Scene {
public:
    std::vector<Object> objects;
    std::vector<Light> lights;

    Scene() = default;
    Scene(std::vector<Object> _objects, std::vector<Light> _lights)
        : objects(std::move(_objects)), lights(std::move(_lights)) {}

};

Scene loadSceneFromJson(const std::map<std::string, Object>& rawObjects, const std::string& fileJSON);

