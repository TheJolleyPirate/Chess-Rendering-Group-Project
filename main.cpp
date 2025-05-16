#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Eigen/Dense"
#include <filesystem>
#include <map>

#include <scene.hpp>
#include <object.hpp>
#include <rasterizer.hpp>
#include <camera.hpp>
#include <shader.hpp>

#include <loadModel.cpp>
#include <shader.cpp>

using namespace std;
using namespace Eigen;
using namespace filesystem;

void loadModel(map<string, Object> &objects, string fileLocation){
    char delim = '/';
    vector<string> fileSplit = split(fileLocation, delim);
    string fileName = fileSplit.back();
    delim = '.';
    fileName = split(fileName, delim)[0];
    Object object = load(fileLocation);
    if(objects.count(fileName) > 0){
        string badInput = "multiple files with the same name";
        throw badInput;
    }
    objects[fileName] = object;
}

map<string, Object> loadModels(vector<string> files){
    map<string, Object> objects;
    for(string file : files){
        loadModel(objects, file);
    }
    return objects;
}

map<string, Object> loadModels(string folder){
    map<string, Object> objects;
    for (auto &entry : directory_iterator(folder)){
        string file = entry.path();
        if(file.substr(file.size() - 4, 4) == ".obj"){
            loadModel(objects, file);
        }
    }
    return objects;
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
    //r.setView(camera.getViewMatrix());
    //r.setProjection(camera.getProjectionMatrix());
    r.setFragmentShader(active_shader);
    // Draw objects
    r.rasterizeObjects(scene);

}

int main(){
    std::map<std::string, Object> objects = loadModels("../Models");
    Scene scene = buildScene(objects);
    renderScene(scene);
    return 0;
}