#include <iostream>
#include <opencv.hpp>
#include "rasterizer.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;
inline double Degree(double angle)  {return angle*MY_PI/180.0;}

/*
**给定旋转角度，确定model矩阵
*/
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    return model;
}

/*
**给定摄像机位置点，确定view矩阵
*/
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

/*
**给定摄像机fov、长宽比ratio、近平面距离、远平面距离，确定projection矩阵
*/
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection;

    float n = zNear;
    float f = zFar;
    float t = -abs(zNear)*tan(Degree(eye_fov)/2.0);//t值根据公式取得
    float r = t * aspect_ratio;//r值根据公式取得

    projection << n/r, 0, 0, 0,
                0, n/t, 0, 0,
                0, 0, (n+f)/(n-f), -2*n*f/(n-f),
                0, 0, 1, 0;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0,0,5};


    std::vector<Eigen::Vector3f> pos//Vector3f数组
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };//元素表示顶点的坐标

    std::vector<Eigen::Vector3i> ind//Vector3i数组
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };//元素表示组成三角形的顶点序列

    std::vector<Eigen::Vector3f> cols//Vector3f数组
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };//元素顺序表示每个顶点的RGB颜色值

    auto pos_id = r.load_positions(pos);//为顶点坐标建立map，pos_id作为键，pos作为值
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //确定MVP变换矩阵
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);//光栅化三角形
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }
    
    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        cv::imwrite("resutl.png",image);
        key = cv::waitKey(10);

    }

    return 0;
}
