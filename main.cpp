#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Eigen/Dense"

#include <scene.hpp>
#include <object.hpp>
#include <headerFiles/rasterizer.hpp>
#include <headerFiles/camera.hpp>
#include <headerFiles/shader.hpp>

#include <src/loadModel.cpp>
#include <src/shader.cpp>

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
    // Manually create camera for now -- potentially add to scene class
    Camera camera(Vector3f(0, 0, 10), Vector3f(0, 0, 0), Vector3f(0, 1, 0), 45.0f, 1.0f, 0.1f, 50.0f);
    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = texture_fragment_shader;

    // Set up rasterizer
    rst::Rasterizer r(512, 512);
    r.clear(rst::Buffers::Colour | rst::Buffers::Depth);
    r.setView(camera.getViewMatrix());
    r.setProjection(camera.getProjectionMatrix());
    r.setFragmentShader(active_shader);
    // Draw objects
    r.rasterizeObjects(scene);

}

int main(){

    return 0;
}