//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <math.h>

class Texture {
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
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

        auto u_0 = std::max(std::floor(u_img - 0.5f), 0.0f);
        auto u_1 = std::min(std::ceil(u_img + 0.5f), width * 1.0f);
        auto v_0 = std::max(std::floor(v_img - 0.5f), 0.0f);
        auto v_1 = std::min(std::ceil(v_img + 0.5f), height * 1.0f);

        auto s = u_img - u_0;
        auto t = v_img - v_0;

        auto c00 = image_data.at<cv::Vec3b>(v_0, u_0);
        auto c01 = image_data.at<cv::Vec3b>(v_0, u_1);
        auto c10 = image_data.at<cv::Vec3b>(v_1, u_0);
        auto c11 = image_data.at<cv::Vec3b>(v_1, u_1);

        auto cl0 = c00 + t * (c10 - c00);
        auto cl1 = c01 + t * (c11 - c01);
        auto color = cl0 + s * (cl1 - cl0);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
