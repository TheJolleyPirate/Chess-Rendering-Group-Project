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
    Light dummyLight = Light(Vector3f(0, 10, 5), Vector3f(1, 1, 1));
    Object dummyObject = objects.at("king");
    return Scene({dummyObject}, {dummyLight}, {0, 0, 0});
}