#include "scene.hpp"
#include "loadMesh.hpp"
#include "json.hpp"
#include "shader.hpp"
#include <fstream>
#include <iostream>
#include <memory>
#include <Eigen/Dense>

using json = nlohmann::json;
// Test if the json file can be loaded correctly
void loadSceneFromJson(const std::string& path, Scene& scene) {
    std::ifstream in(path);
    if (!in.is_open()) {
        std::cerr << "Failed to open scene JSON: " << path << std::endl;
        return;
    }

    json j;
    in >> j;

    for (const auto& obj : j["objects"]) {
        std::string mesh_path = obj["mesh"];
        std::shared_ptr<Mesh> mesh = loadModel(mesh_path);
        if (!mesh) {
            std::cerr << "Failed to load mesh from: " << mesh_path << std::endl;
            continue;
        }

        // Default transform: identity
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        if (obj.contains("transform")) {
            auto t = obj["transform"];
            if (t.contains("translate")) {
                Eigen::Vector3f tr(t["translate"][0], t["translate"][1], t["translate"][2]);
                transform.translate(tr);
            }
            if (t.contains("scale")) {
                Eigen::Vector3f s(t["scale"][0], t["scale"][1], t["scale"][2]);
                transform.scale(s);
            }
            if (t.contains("rotate")) {
                float angle = t["rotate"]["angle"];
                Eigen::Vector3f axis(t["rotate"]["axis"][0], t["rotate"]["axis"][1], t["rotate"]["axis"][2]);
                transform.rotate(Eigen::AngleAxisf(angle, axis.normalized()));
            }
        }

        Material mat;
        if (obj.contains("material")) {
            auto m = obj["material"];
            mat.ambient = Eigen::Vector3f(m["ambient"][0], m["ambient"][1], m["ambient"][2]);
            mat.diffuse = Eigen::Vector3f(m["diffuse"][0], m["diffuse"][1], m["diffuse"][2]);
            mat.specular = Eigen::Vector3f(m["specular"][0], m["specular"][1], m["specular"][2]);
            mat.shininess = m["shininess"];
        }

        scene.addObject(mesh, mat, transform);
    }
}
