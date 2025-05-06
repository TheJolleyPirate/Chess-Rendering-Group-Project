#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Eigen/Dense"

#include <scene.hpp>
#include <light.hpp>
#include <object.hpp>

using namespace std;
using namespace Eigen;

void setPixel(Vector2i position, Vector3f colour){

}

//not sure what return type for this should be yet
Vector3f getTexture(Vector2i textureCoords, string textureFile){

    return {0, 0, 0};
}

shared_ptr<Face> transformToViewSpace(shared_ptr<Face> face){
    
    return nullptr;
}

//if we know the face is a triangle
void rastorizeTriangle(Face face){

}

//if we don't know what shape the face is
void rastorizeFace(Face face){
    
}

void rastorizeObjects(Scene scene){

}