#include <vector>
#include <Eigen/Dense>
#include <headerFiles/light.hpp>
#include <headerFiles/object.hpp>

class Scene;

class Scene{
    public:
        std::vector<Object> objects;
        std::vector<Light> lights;
        Eigen::Vector3f eyePosition;
        Scene(std::vector<Object> _objects, std::vector<Light> _lights, Eigen::Vector3f _eyePosition){
            objects = _objects;
            lights = _lights;
            eyePosition = _eyePosition;
        }
        void buildScene();
};