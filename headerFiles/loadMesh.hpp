// loadModel.hpp
#pragma once
#include <memory>
#include <string>
#include "mesh.hpp"

std::shared_ptr<Mesh> loadModel(const std::string& filename);
