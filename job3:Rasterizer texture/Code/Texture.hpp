#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv.hpp>
class Texture{
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

    /*
    **UV坐标系的原点在左下角，而给定float类型的u、v值原点在左上角
    */
    Eigen::Vector3f getColor(float u, float v)
    {
        u = std::min(1.0f, std::max(0.0f, u));//保证u的取值在[0.,1.]内
        v = std::min(1.0f, std::max(0.0f, v));

        auto u_img = u * width;//u对应的U轴坐标值 
        auto v_img = (1 - v) * height;//v对应的V轴坐标值
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);//取得该坐标点处的color值
        return Eigen::Vector3f(color[0], color[1], color[2]);//以Vector3f类型返回color值
    }

    /*
    **双线性插值
    */
    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        u=std::min(1.0f, std::max(0.0f, u));
        v=std::min(1.0f, std::max(0.0f, v));
        
        auto u_img = u * width; 
        auto v_img = (1 - v) * height;

        auto u_min = std::floor(u_img);//？
        auto u_max = std::ceil(u_img);
        auto v_min = std::floor(v_img);
        auto v_max = std::ceil(v_img);

        // Color_11 Color_12
        // Color_21 Color_22
        // in opencv, higher y in image->smaller v in texture
        auto color_11=image_data.at<cv::Vec3b>(v_min,u_min);
        auto color_12=image_data.at<cv::Vec3b>(v_min,u_max);
        auto color_21=image_data.at<cv::Vec3b>(v_max,u_min);
        auto color_22=image_data.at<cv::Vec3b>(v_max,u_max);

        auto color1=(u_img-u_min)*color_11+(u_max-u_img)*color_12;
        auto color2=(u_img-u_min)*color_21+(u_max-u_img)*color_22;

        auto color=(v_img-v_min)*color1+(v_max-v_img)*color2;

        return Eigen::Vector3f(color[0], color[1], color[2]);
    } 

};
#endif //RASTERIZER_TEXTURE_H
