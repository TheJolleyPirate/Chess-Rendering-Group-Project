#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <map>
#include "Eigen/Dense"

#include <scene.hpp>
#include <light.hpp>
#include <object.hpp>

using namespace std;
using namespace Eigen;

Scene buildScene(std::map<std::string, Object> objects){
    Light dummy = Light();
    Object object = objects.at("king");
    return Scene(vector<Object>(), vector<Light>(), {0, 0, 0});
}