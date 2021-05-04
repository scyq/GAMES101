//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture
{
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        float u_min = std::floor(u_img);
        float u_max = std::min((float)width, std::ceil(u_img));
        float v_min = std::floor(v_img);
        float v_max = std::min((float)height, std::ceil(v_img));

        auto left_top = image_data.at<cv::Vec3b>(v_min, u_min);
        auto left_bottom = image_data.at<cv::Vec3b>(v_max, u_min);
        auto right_top = image_data.at<cv::Vec3b>(v_min, u_max);
        auto right_bottom = image_data.at<cv::Vec3b>(v_max, u_max);

        float u_ratio = (u_img - u_min) / (u_max - u_min);
        auto lerp_top = (1 - u_ratio) * left_top + u_ratio * right_top;
        auto lerp_bottom = (1 - u_ratio) * left_bottom + u_ratio * right_bottom;

        float v_ratio = (v_img - v_max) / (v_min - v_max);
        auto bilerp_color = v_ratio * lerp_top + (1 - v_ratio) * lerp_bottom;

        return Eigen::Vector3f(bilerp_color[0], bilerp_color[1], bilerp_color[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
