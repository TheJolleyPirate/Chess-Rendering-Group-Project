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
#include "pathtracer.hpp"

#include <loadModel.cpp>
#include <shader.cpp>

using namespace std;
using namespace Eigen;
using namespace filesystem;

void loadModel(map<string, Object> &objects, string fileLocation){
    cout << "loading model: " << fileLocation << "\n";
    char delim = '/';
    vector<string> fileSplit = split(fileLocation, delim);
    string fileName = fileSplit.back();
    delim = '.';
    fileName = split(fileName, delim)[0];
    if(objects.count(fileName) > 0){
        string badInput = "multiple files with the same name";
        throw badInput;
    }
    cout << "mesh name: " << fileName << "\n";
    Object object = load(fileLocation, fileName);
    cout << "successfully retreived object!\n";
    objects[fileName] = object;
}

map<string, Object> loadModels(vector<string> files){
    map<string, Object> objects;
    for(string file : files){
        loadModel(objects, file);
    }
    return objects;
}

void getFiles(vector<string> &files, string folder){
    for (auto &entry : directory_iterator(folder)){
        if(entry.is_directory()){
            getFiles(files, entry.path());
        }
        else{
            string file = entry.path();
            if(file.substr(file.size() - 4, 4) == ".obj"){
                files.push_back(file);
            }
        }
    }
}

map<string, Object> loadModels(string folder){
    map<string, Object> objects;
    vector<string> files;
    getFiles(files, folder);
    for(string file : files){
        loadModel(objects, file);
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
    float height = 512;
    float width = 512;
    rst::Rasterizer r(width, height);
    r.clear(rst::Buffers::Colour | rst::Buffers::Depth);
    r.setView(camera.getViewMatrix());
    r.setProjection(camera.getProjectionMatrix());
    r.setFragmentShader(active_shader);
    // Draw objects
    r.rasterizeObjects(scene);
    cv::Mat image(width, height, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    cv::imshow("image", image);
    cv::imwrite("output.png", image);

}

int RENDER_METHOD = 1; // 0 for rasterization, 1 for ray tracing

int main() {
    if (RENDER_METHOD == 0) {
        try{
            cout << "loading models\n";
            std::map<std::string, Object> objects = loadModels("../Models/");
            cout << "models loaded\n";
            cout << "building scene\n";
            Scene scene = loadSceneFromJson(objects, "../Models/chess_scene.json");
            cout << "scene built\n";
            cout << "rendering scene\n";
            auto start = std::chrono::system_clock::now();
            renderScene(scene);
            auto stop = std::chrono::system_clock::now();
            cout << "scene rendered\n";
            auto time = stop - start;
            auto hours = std::chrono::duration_cast<std::chrono::hours>(time);
            time = time - hours;
            auto minutes = std::chrono::duration_cast<std::chrono::minutes>(time);
            time = time - minutes;
            auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time).count();
            cout << "render time: " << hours.count() << ":" << minutes.count() << ":" << seconds << "\n";
        }
        catch(string message){
            cerr << "exception occurred, message: " << message << "\n";
        }    
    return 0;
    } else {
        try {
            std::cout << "Path Tracer" << std::endl;
            std::cout << "Loading models..." << std::endl;
            auto objects = loadModels("../Models/");
            std::cout << "Building scene..." << std::endl;
            Scene scene = loadSceneFromJson(objects, "../Models/chess_scene.json");
            std::cout << "Initialising camera..." << std::endl;
            Camera cam({0, 10, -10}, {0, 0, 0}, {0, 1, 0}, 45.0f, 1.0f, 0.1f, 100.0f);

            int width = 512, height = 512, spp = 1;
            PathTracer PathTracer(width, height, spp);

            std::cout << "Rendering scene..." << std::endl;
            auto start = std::chrono::high_resolution_clock::now();
            PathTracer.render(scene, cam, "output");
            auto stop = std::chrono::high_resolution_clock::now();

            double seconds = std::chrono::duration_cast<std::chrono::seconds>(stop - start).count();
            std::cout << "Render completed in " << seconds << "s!" << std::endl;
        } catch (const std::string &message) {
                std::cerr << "Error: " << message << std::endl;
                return 1;
        }
    return 0;
    }
}