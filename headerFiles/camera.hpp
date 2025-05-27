#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <Eigen/Dense>
#include "ray.hpp"

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

    Camera(Eigen::Vector3f &_position, Eigen::Vector3f &_target, Eigen::Vector3f &_up, float _fov, float _aspectRatio);

    Ray getRay(float u, float v) const;

    Eigen::Matrix4f lookAt() const {
        Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

        Eigen::Vector3f z = (position - target).normalized();
        Eigen::Vector3f x = up.cross(z).normalized();
        Eigen::Vector3f y = z.cross(x);
    
        Eigen::Matrix4f rotate;
        rotate << x.x(), x.y(), x.z(), 0, 
                    y.x(), y.y(), y.z(), 0, 
                    z.x(), z.y(), z.z(), 0, 
                    0, 0, 0, 1;
    
        Eigen::Matrix4f translate;
        translate << 1, 0, 0, -position[0], 
                        0, 1, 0, -position[1], 
                        0, 0, 1, -position[2], 
                        0, 0, 0, 1;
    
        view = rotate * translate * view;
        return view;
    }
    
    // Get the view matrix for the camera
    Eigen::Matrix4f getViewMatrix() const {
        Eigen::Matrix4f view = lookAt();
        return view;
    }
    
    // Compute the projection matrix using the camera's parameters
    Eigen::Matrix4f getProjectionMatrix() const {
        Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    
        float fov_rad = fov * 3.1415926 / 180.0;
        float top = nearPlane * tan(fov_rad / 2.0);
        float bottom = -top;
        float right = top * aspectRatio;
        float left = -right;
    
        projection << nearPlane / right, 0, 0, 0, 
                        0, nearPlane / top, 0, 0, 
                        0, 0, (nearPlane + farPlane) / (nearPlane - farPlane), 2 * nearPlane * farPlane / (nearPlane - farPlane), 
                        0, 0, -1, 0;
    
        return projection;
    }

private:
    Eigen::Vector3f origin_;
    Eigen::Vector3f lower_left_;
    Eigen::Vector3f horizontal_;
    Eigen::Vector3f vertical_;
};

#endif // CAMERA_HPP