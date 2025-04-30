#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/core/base.hpp>
#include <opencv2/opencv.hpp>

#include "Triangle.hpp"
#include "mesh.hpp"
#include "rasterizer.hpp"
#include <cmath>

constexpr double MY_PI = 3.1415926;

// TODO: Implement this function.
Eigen::Matrix4f get_translation(const Eigen::Vector3f &translation) {
    // Calculate a transformation matrix of given translation vector.
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans.col(3).row(0).setConstant(translation.x());
    trans.col(3).row(1).setConstant(translation.y());
    trans.col(3).row(2).setConstant(translation.z());
    return trans;
}

// TODO: Implement this function.
/**
 * @brief Get the rotation transformation matrix given rotation angle and axis.
 *
 * @param rotation_angle: rotation angle in degree.
 * @param axis: rotation axis.
 */
Eigen::Matrix4f get_rotation(float rotation_angle, const Eigen::Vector3f &axis) {
    Eigen::Matrix4f rotation_matrix;
    float x1, x2, x3, y1, y2, y3, z1, z2, z3;
    float radians_angle = rotation_angle * MY_PI / 180;
    Eigen::Vector3f norm = axis.normalized();
    float cosine = cos(radians_angle);
    float sine = sin(radians_angle);
    float x = norm.x();
    float y = norm.y();
    float z = norm.z();
    x1 = cosine + (x * x * (1 - cosine));
    x2 = (x * y * (1 - cosine)) - (z * sine);
    x3 = (x * z * (1 - cosine)) + (y * sine);
    y1 = (y * x * (1 - cosine)) + (z * sine);
    y2 = cosine + (y * y * (1 - cosine));
    y3 = (y * z * (1 - cosine)) - (x * sine);
    z1 = (z * x * (1 - cosine)) - (y * sine);
    z2 = (z * y * (1 - cosine)) + (x * sine);
    z3 = cosine + (z * z * (1 - cosine));
    rotation_matrix << x1, x2, x3, 0, y1, y2, y3, 0, z1, z2, z3, 0, 0, 0, 0, 1;
    return rotation_matrix;
}

// TODO: Implement this function
/**
 * @brief Get the view matrix by given eye position, target view position and up vector.
 *
 * @param eye_pos: location of the camera
 * @param target: the point the camera is looking at
 */
Eigen::Matrix4f look_at(Eigen::Vector3f eye_pos, Eigen::Vector3f target,
                        Eigen::Vector3f up = Eigen::Vector3f(0, 1, 0)) {
    //I found a look_at algorithm at https://www.3dgep.com/understanding-the-view-matrix/#The_View_Matrix
    Eigen::Vector3f zAxis = (eye_pos - target).normalized();
    Eigen::Vector3f xAxis = up.cross(zAxis).normalized();
    Eigen::Vector3f yAxis = zAxis.cross(xAxis);

    Eigen::Matrix4f rotate, translate, view;

    rotate << xAxis.x(), yAxis.x(), zAxis.x(), 0, xAxis.y(), yAxis.y(), zAxis.y(), 0, xAxis.z(), yAxis.z(), zAxis.z(), 0, 0, 0, 0, 1;
    translate << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, -eye_pos.x(), -eye_pos.y(), -eye_pos.z(), 1;
    view = rotate * translate;
    return view;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
    Eigen::Matrix4f view;

    view = look_at(eye_pos, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 1, 0));

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle, const Eigen::Vector3f &axis,
                                 const Eigen::Vector3f &translation) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f rotation = get_rotation(rotation_angle, axis);

    Eigen::Matrix4f trans = get_translation(translation);

    model = trans * rotation * model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fovy, float aspect_ratio, float zNear, float zFar) {
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // Perspective to orthographic projection
    Eigen::Matrix4f persp_to_ortho = Eigen::Matrix4f::Identity();
    persp_to_ortho << zNear, 0, 0, 0, 0, zNear, 0, 0, 0, 0, -(zNear + zFar), -zNear * zFar, 0, 0, -1, 0;

    // translate to origin
    float eye_fovy_rad = eye_fovy * MY_PI / 180.0;
    float top = zNear * tan(eye_fovy_rad / 2.0);
    float bottom = -top;
    float right = top * aspect_ratio;
    float left = -right;

    Eigen::Matrix4f ortho_translate = Eigen::Matrix4f::Identity();
    ortho_translate << 1, 0, 0, -(left + right) / 2., 0, 1, 0, -(bottom + top) / 2., 0, 0, 0, -(zNear + zFar) / 2., 0,
        0, 0, 1;

    // scale to NDC
    Eigen::Matrix4f ortho_scale = Eigen::Matrix4f::Identity();
    ortho_scale << 2. / (right - left), 0, 0, 0, 0, 2. / (top - bottom), 0, 0, 0, 0, 2. / (zFar - zNear), 0, 0, 0, 0, 1;

    projection = ortho_scale * ortho_translate * persp_to_ortho * projection;

    return projection;
}

int main(int argc, const char **argv) {
    std::cout << "running\n";
    float angle = 0;
    Eigen::Vector3f axis = Eigen::Vector3f(0, 1, 0);
    Eigen::Vector3f translation = Eigen::Vector3f(0, 0, 0);
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]);  // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        } else
            return 0;
    }

    // load house mesh
    Mesh house_mesh;
    if (!house_mesh.load_obj("/home/danny/Desktop/Projects/computer graphics/COMP46108610-2025-HW1-20250228/model/house.obj")) {
        std::cerr << "Failed to load house mesh." << std::endl;
        return -1;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 15};

    auto pos_id = r.load_positions(house_mesh.vertices);
    auto ind_id = r.load_indices(house_mesh.faces);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        Eigen::Matrix4f temp = get_model_matrix(angle, axis, translation);
        //std::cout << "model matrix:\n" << temp << "\n";
        r.set_model(temp);
        temp = get_view_matrix(eye_pos);
        //std::cout << "view matrix:\n" << temp << "\n";
        r.set_view(temp);
        temp = get_projection_matrix(45, 1, 0.1, 50);
        //std::cout << "projection matrix:\n" << temp << "\n";
        r.set_projection(temp);
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }
    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle, axis, translation));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            eye_pos.x() -= 1;
        } else if (key == 'd') {
            eye_pos.x() += 1;
        } else if (key == 'w') {
            eye_pos.y() += 1;
        } else if (key == 's') {
            eye_pos.y() -= 1;
        } else if (key == 'q') {
            eye_pos.z() -= 1;
        } else if (key == 'e') {
            eye_pos.z() += 1;
        } else if (key == 'j') {
            angle += 10;
        } else if (key == 'k') {
            angle -= 10;
        }
        std::cout << "eye_pos: " << eye_pos.transpose() << std::endl;
    }
    return 0;
}
