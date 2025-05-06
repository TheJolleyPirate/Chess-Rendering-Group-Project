#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Eigen/Dense"

#include <scene.hpp>
#include <object.hpp>

#include <loadModel.cpp>
#include <renderer.cpp>

using namespace std;
using namespace Eigen;

vector<Object> loadModel(string fileLocation){

    return vector<Object>();
}

vector<Object> loadModels(vector<string> files){

    return vector<Object>();
}

vector<Object> loadModels(string folder){

    return vector<Object>();
}

void captureImage(){
    
}

Matrix4f rotateScene(float angle, Vector3f axis){
    
    return Matrix4f::Identity();
}

void renderScene(Scene scene){

}

int main(){

    return 0;
}