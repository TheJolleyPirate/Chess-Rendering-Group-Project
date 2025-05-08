#include <cmath>

#include <headerFiles/camera.hpp>

// Compute the view matrix using the camera's position, target, and up vector
Eigen::Matrix4f Camera::lookAt() const {
    Eigen::Matrix4f view;
    
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
Eigen::Matrix4f Camera::getViewMatrix() const {
    Eigen::Matrix4f view = lookAt();
    return view;
}

// Compute the projection matrix using the camera's parameters
Eigen::Matrix4f Camera::getProjectionMatrix() const {
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