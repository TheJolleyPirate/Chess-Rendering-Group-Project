#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Eigen/Dense"

#include <headerFiles/scene.hpp>
#include <headerFiles/light.hpp>
#include <headerFiles/object.hpp>

using namespace std;
using namespace Eigen;

void setPixel(Vector2i position, Vector3f colour){

}

//not sure what return type for this should be yet
Vector3f getTexture(Vector2i textureCoords, string textureFile){

}

Face transformToViewSpace(Face face){

}

//if we know the face is a triangle
void rastorizeTriangle(Face face){

}

//if we don't know what shape the face is
void rastorizeFace(Face face){
    
}

void rastorizeObjects(Scene scene){

}