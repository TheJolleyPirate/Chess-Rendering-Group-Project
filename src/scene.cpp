#include <scene.hpp>
#include <json.hpp>
#include <fstream>
#include <Eigen/Dense>

using json = nlohmann::json;
using namespace Eigen;

/*by Daniel Jolley-Rogers u7511912
applies a Eigen transform to an object*/
void applyTransform(Object &object, const Affine3f &transform) {
    Matrix4f T = transform.matrix();
    for (auto& v : object.vertices) {
        Vector4f hp;
        hp.head<3>() = v->position;
        hp[3] = 1.0f;
        v->position = (T * hp).head<3>();
    }
}

/*by Yue Zhu u7442130
refactored by Daniel Jolley-Rogers u7511912
uses a JSON file to construct the scene*/
Scene loadSceneFromJson(const std::map<std::string, Object>& rawObjects, const std::string& fileJSON) {
    std::ifstream in(fileJSON);
    if (!in.is_open()) {
        throw std::runtime_error("Cannot open JSON scene file: " + fileJSON);
    }

    json j;
    in >> j;
    in.close();

    std::vector<Object> sceneObjects;

    for (const auto& objectSceneInfo : j["objects"]) {
        std::string modelName = objectSceneInfo["model"];
        if (rawObjects.count(modelName) == 0) {
            throw std::runtime_error("No mesh named: " + modelName);
        }
        Object object = rawObjects.at(modelName);

        Affine3f transform = Affine3f::Identity();
        if (objectSceneInfo.contains("transform")) {
            const auto& t = objectSceneInfo["transform"][0];
            if (t.contains("translate")) {
                transform.translate(Vector3f(t["translate"][0], t["translate"][1], t["translate"][2]));
            }
            if (t.contains("rotate")) {
                float angle = t["rotate"][0]["angle"][0];
                Vector3f axis(t["rotate"][0]["axis"][0], t["rotate"][0]["axis"][1], t["rotate"][0]["axis"][2]);
                transform.rotate(AngleAxisf(angle, axis.normalized()));
            }
            if (t.contains("scale")) {
                transform.scale(Vector3f(t["scale"][0], t["scale"][1], t["scale"][2]));
            }
        }

        applyTransform(object, transform);
        sceneObjects.push_back(object);
    }

    // default light
    std::vector<Light> lights = {
        Light(Vector3f(0, 10, 0), Vector3f(1, 1, 1))
    };

    return Scene(sceneObjects, lights);
}
