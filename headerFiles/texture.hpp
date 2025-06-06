#pragma once

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

// From CLab2
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        if (name != "") {
            image_data = cv::imread(name);
            cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
            width = image_data.cols;
            height = image_data.rows;
        }
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v) const
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};