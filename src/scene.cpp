#include "scene.hpp"
#include "json.hpp"
#include "shader.hpp"
#include <fstream>
#include <iostream>
#include <memory>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
using json = nlohmann::json;

void applyTransform(Object &object, const Affine3f &transform){
    Matrix4f transformationMatrix = transform.matrix();
    for(shared_ptr<Vertex> vertex : object.vertices){
        Vector4f homoPosition;
        homoPosition.head(3) = vertex->position;
        homoPosition(3) = 1;
        vertex->position = (transformationMatrix * homoPosition).head(3);
    }
}

Scene loadSceneFromJson(std::map<std::string, Object> rawObjects, std::string fileJSON){
    ifstream in(fileJSON);
    // Test if the json file can be loaded correctly
    if (!in.is_open()) {
        string message = "Failed to open scene JSON: " + fileJSON;
        throw message;
    }
    json j;
    in >> j;
    vector<Object> sceneObjects;

    for (const auto& objectSceneInfo : j["objects"]) {
        string objectName = objectSceneInfo["model"];
        if(rawObjects.count(objectName) == 0){
            string message = "no mesh named " + objectName;
            throw(message);
        }
        Object object = rawObjects[objectName];

        // Default transform: identity
        Affine3f transform = Affine3f::Identity();

        if (objectSceneInfo.contains("transform")) {
            auto transformInfo = objectSceneInfo["transform"];
            if (transformInfo.contains("translate")) {
                Vector3f tr(transformInfo["translate"][0], transformInfo["translate"][1], transformInfo["translate"][2]);
                transform.translate(tr);
            }
            if (transformInfo.contains("rotate")) {
                float angle = transformInfo["rotate"]["angle"];
                Vector3f axis(transformInfo["rotate"]["axis"][0], transformInfo["rotate"]["axis"][1], transformInfo["rotate"]["axis"][2]);
                transform.rotate(AngleAxisf(angle, axis.normalized()));
            }
            if (transformInfo.contains("scale")) {
                Vector3f s(transformInfo["scale"][0], transformInfo["scale"][1], transformInfo["scale"][2]);
                transform.scale(s);
            }
        }
        applyTransform(object, transform);
        sceneObjects.push_back(object);
    }
    in.close();
    Light light = Light({0, 10, 0}, {1, 1, 1});
    return Scene(sceneObjects, {light});
}