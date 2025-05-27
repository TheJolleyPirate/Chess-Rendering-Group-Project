#include "camera.hpp"
#include <cmath>

// Create camera for path tracing
Camera::Camera(Eigen::Vector3f &_position, Eigen::Vector3f &_target, Eigen::Vector3f &_up, float _fov, float _aspectRatio) {
    float theta = _fov * M_PI / 180.0f;
    float half_height = std::tan(theta / 2.0f);
    float half_width = half_height * _aspectRatio;
    Eigen::Vector3f w = (_position - _target).normalized();
    Eigen::Vector3f u = _up.cross(w).normalized();
    Eigen::Vector3f v = w.cross(u);

    origin_ = _position;
    horizontal_ = 2.0f * half_width * u;
    vertical_ = 2.0f * half_height * v;
    lower_left_ = origin_ - (horizontal_ / 2.0f) - (vertical_ / 2.0f) - w;
}

Ray Camera::getRay(float u, float v) const {
    Eigen::Vector3f direction = lower_left_ + u * horizontal_ + v * vertical_ - origin_;
    return Ray(origin_, direction.normalized());
}