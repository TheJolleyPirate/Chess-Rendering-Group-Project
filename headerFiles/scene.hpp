#pragma once

#include <vector>
#include <Eigen/Dense>
#include <light.hpp>
#include <object.hpp>
#include <string>
#include <map>

class Scene;

class Scene{
    public:
        std::vector<Object> objects;
        std::vector<Light> lights;
        Scene(std::vector<Object> _objects, std::vector<Light> _lights){
            objects = _objects;
            lights = _lights;
        }
};
Scene loadSceneFromJson(std::map<std::string, Object> objects, std::string fileJSON);