#pragma once

#include <Eigen/Dense>

// This class represents a camera in 3D space, with position, target, up vector, field of view (FOV), aspect ratio, and near/far planes.
class Camera {
public:
    Eigen::Vector3f position;
    Eigen::Vector3f target;
    Eigen::Vector3f up;
    float fov;
    float aspectRatio;
    float nearPlane;
    float farPlane;

    Camera(Eigen::Vector3f _position, Eigen::Vector3f _target, Eigen::Vector3f _up, float _fov, float _aspectRatio, float _nearPlane, float _farPlane)
        : position(_position), target(_target), up(_up), fov(_fov), aspectRatio(_aspectRatio), nearPlane(_nearPlane), farPlane(_farPlane) {}

    Eigen::Matrix4f lookAt() const;
    Eigen::Matrix4f getViewMatrix() const;
    Eigen::Matrix4f getProjectionMatrix() const;
};