#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/core/base.hpp>
#include <opencv2/opencv.hpp>

#include "Triangle.hpp"
#include "mesh.hpp"
#include "rasterizer.hpp"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_translation(const Eigen::Vector3f &translation) {
    // Calculate a transformation matrix of given translation vector.
    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
    trans.col(3).row(0).setConstant(translation.x());
    trans.col(3).row(1).setConstant(translation.y());
    trans.col(3).row(2).setConstant(translation.z());
    return trans;
}

/**
 * @brief Get the rotation transformation matrix given rotation angle and axis.
 *
 * @param rotation_angle: rotation angle in degree.
 * @param axis: rotation axis.
 */
Eigen::Matrix4f get_rotation(float rotation_angle, const Eigen::Vector3f &axis) {
    Eigen::Matrix4f rotation_matrix;
    float x1, x2, x3, y1, y2, y3, z1, z2, z3;
    //need angle in radians to work with cmath
    float radians_angle = rotation_angle * MY_PI / 180;
    //make sure the axis vector is normalised
    Eigen::Vector3f norm = axis.normalized();
    float cosine = cos(radians_angle);
    float sine = sin(radians_angle);
    float x = norm.x();
    float y = norm.y();
    float z = norm.z();
    //setting up the different spots in the matrix according to the angle - axis formula
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

Eigen::Matrix4f get_scaling(const Eigen::Vector3f &scaling) {
    //already works as written, no need to change
    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    scale(0, 0) = scaling.x();
    scale(1, 1) = scaling.y();
    scale(2, 2) = scaling.z();
    return scale;
}

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
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    view = look_at(eye_pos, Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 1, 0));

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle, const Eigen::Vector3f &axis,
                                 const Eigen::Vector3f &translation, const Eigen::Vector3f &s = {0, 0, 0}) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f rotation = get_rotation(rotation_angle, axis);

    Eigen::Matrix4f trans = get_translation(translation);
    //added scaling to the get model function for convenience
    Eigen::Matrix4f scale = get_scaling(s);

    model = trans * rotation * scale * model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fovy, float aspect_ratio, float zNear, float zFar) {
    // Create the projection matrix for the given parameters.
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

//quality of life function
void operator+= (Mesh &a, Mesh b){
    a = a + b;
}

Mesh create_cubism_mesh(const Mesh &cube) {
    //quick lambda to simplify making meshes
    auto getMesh = [&](float rotation_angle, const Eigen::Vector3f &axis,
        const Eigen::Vector3f &translation, const Eigen::Vector3f &s = {0, 0, 0}){
            Eigen::Matrix4f modelMatrix = get_model_matrix(rotation_angle, axis, translation, s);
            Mesh m = cube.transform(modelMatrix);
            return m;
        };
    //front body
    Mesh cubism = getMesh(-5, {0, 0, 1}, {0, 2, 0}, {3, 2, 1});
    //back body
    cubism += getMesh(5, {0, 0, 1}, {-2.8, 2.1, 0}, {3, 1.7, 0.8});
    
    //front right leg
    cubism += getMesh(0, {0, 0, 0}, {0.8, 0, 0.7}, {0.6, 3.8, 0.5});
    //front right foot
    cubism += getMesh(0, {0, 0, 0}, {1.2, -1.8, 0.7}, {0.4, 0.3, 0.6});

    //front left leg
    cubism += getMesh(0, {0, 0, 0}, {0.8, 0, -0.7}, {0.6, 3.8, 0.5});
    //front left foot
    cubism += getMesh(0, {0, 0, 0}, {1.2, -1.8, -0.7}, {0.4, 0.3, 0.6});
    
    //right back hip
    cubism += getMesh(-5, {0, 0, 1}, {-3.8, 1.5, 0.6}, {0.8, 1.5, 0.5});
    //right back thigh
    cubism += getMesh(-15, {0, 0, 1}, {-3.95, 0.3, 0.6}, {0.6, 1.5, 0.5});
    //right back shin
    cubism += getMesh(-2.5, {0, 0, 1}, {-4.15, -1, 0.6}, {0.5, 1.5, 0.5});
    //right back foot
    cubism += getMesh(0, {0, 0, 0}, {-3.8, -1.8, 0.6}, {0.4, 0.3, 0.6});

    //left back hip
    cubism += getMesh(-5, {0, 0, 1}, {-3.8, 1.5, -0.6}, {0.8, 1.5, 0.5});
    //left back thigh
    cubism += getMesh(-15, {0, 0, 1}, {-3.95, 0.3, -0.6}, {0.6, 1.5, 0.5});
    //left back shin
    cubism += getMesh(-2.5, {0, 0, 1}, {-4.15, -1, -0.6}, {0.5, 1.5, 0.5});
    //left back foot
    cubism += getMesh(0, {0, 0, 0}, {-3.8, -1.8, -0.6}, {0.4, 0.3, 0.6});

    //neck
    cubism += getMesh(-10, {0, 0, 1}, {1, 3.5, 0}, {1, 1.5, 0.8});
    //muzzle
    cubism += getMesh(-10, {0, 0, 1}, {2, 3.5, 0}, {1, 1.2, 1});
    //forehead
    cubism += getMesh(5, {0, 0, 1}, {1.2, 4, 0}, {1, 0.75, 1.2});
    //nose
    cubism += getMesh(-30, {0, 0, 1}, {2.6, 3.9, 0}, {0.4, 0.2, 0.2});
    //right ear
    cubism += getMesh(-10, {1, 0, 0}, {1, 3.8, 0.6}, {0.6, 1, 0.2});
    //left ear
    cubism += getMesh(10, {1, 0, 0}, {1, 3.8, -0.6}, {0.6, 1, 0.2});

    //upper tail segment
    cubism += getMesh(-60, {0, 0, 1}, {-4.5, 2.3, 0}, {0.4, 1.5, 0.4});
    //middle tail segment
    cubism += getMesh(-40, {0, 0, 1}, {-5.3, 1.6, 0}, {0.3, 1.5, 0.3});
    //lower tail segment
    cubism += getMesh(-20, {0, 0, 1}, {-5.8, 0.6, 0}, {0.2, 1.5, 0.2});


    return cubism;
}

int main(int argc, const char **argv) {
    float angle = 30;
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

    // load mesh primitives
    Mesh cube_mesh;
    cube_mesh.load_obj("../model/cube.obj");
    auto cubism_mesh = create_cubism_mesh(cube_mesh);
    cubism_mesh.save_obj("cubism.obj");

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 2, 15};

    auto pos_id = r.load_positions(cubism_mesh.vertices);
    auto ind_id = r.load_indices(cubism_mesh.faces);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle, axis, translation));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

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
