#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;
inline double Degree(double angle)  {return angle*MY_PI/180.0;}//角度转弧度

/*
**给定绕Z轴的旋转角度，返回用于旋转的变换矩阵model
**此处不要求写平移和缩放
**model矩阵功能：从local space转换到world space
*/
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    double degree = Degree(rotation_angle);
    model << cos(degree), -sin(degree), 0, 0,
            sin(degree), cos(degree), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;//绕Z轴旋转
    return model;
}

/*
**给定eye的位置eye_pos，返回用于平移的变换矩阵view
**此处不要求旋转，即默认了摄像机的位置是形参eye_pos，lookat方向为-z轴，up方向为+y轴
**view矩阵功能：从world space转换到view space
*/
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
                 0, 1, 0, -eye_pos[1],
                 0, 0, 1, -eye_pos[2], 
                 0, 0, 0, 1;
    view = translate * view;

    return view;
}

/*
**给定垂直可视角度eye_fov，长宽比aspect_ratio，近平面z坐标绝对值zNear，远平面z坐标绝对值zFar
*/
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    float f, n, l, r, b, t, fov;
    fov = eye_fov / 180 * MY_PI;//弧度表示的eye_fov
    n = -zNear;//near平面对应的z轴坐标值
    f = -zFar;//near平面对应的z轴坐标值
    t = tan(fov/2) * zNear;//top平面对应的y轴坐标值(height的一半)
    b = -t;//bottom平面对应的y轴坐标值(负值)
    r = t * aspect_ratio;//right平面对应的x轴坐标值(weight的一半)
    l = -r;//left平面对应的x轴坐标值(负值)

    //透视->正交 perspective->orthographic
    Eigen::Matrix4f pertoorth;
    pertoorth << n, 0, 0, 0,
                 0, n, 0, 0,
                 0, 0, n + f, -n*f,
                 0, 0, 1, 0;

    //正交——平移
    Eigen::Matrix4f orth1;
    orth1 << 1, 0, 0, -(r + l) / 2,
             0, 1, 0, -(t + b) / 2,
             0, 0, 1, -(n + f) / 2,
             0, 0, 0, 1;
    //正交——缩放
    Eigen::Matrix4f orth2;
    orth2 << 2 / (r - l), 0, 0, 0,
             0, 2 / (t - b), 0, 0,
             0, 0, 2 / (n - f), 0,
             0, 0, 0, 1;

    projection = orth2 * orth1 * pertoorth;//从右到左
    return projection;

}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);//screen space分辨率

    Eigen::Vector3f eye_pos = {0, 0, 5};//单独给出的eye position坐标(world space)

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};//顶点坐标

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};//三角形顶点连接顺序

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));//0.1是zNear值，50是zFar值，此处均取正值

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        cv::imwrite("resutl.png",image);
        key = cv::waitKey(10);

        // std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
