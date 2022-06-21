#include <iostream>
#include <opencv.hpp>
#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

inline double Degree(double angle)  {return angle*MY_PI/180.0;}


Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();//四维单位矩阵

    Eigen::Matrix4f translate;//将摄像机位置作为原点的变换矩阵
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;
    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    angle = angle * MY_PI / 180.f;
    Eigen::Matrix4f rotation;//绕y轴旋转angle弧度的变换矩阵
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;//x、y、z坐标值均缩放2.5倍的变换矩阵
    scale << 2.5, 0, 0, 0,
             0, 2.5, 0, 0,
             0, 0, 2.5, 0,
             0, 0, 0, 1;

    Eigen::Matrix4f translate;//单位矩阵
    translate << 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;

    return translate * rotation * scale;
}

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

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    //设置blinn phong reflection parameters
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    //两个光源
    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;//specluar幂值

    Eigen::Vector3f color = payload.color;//shading point处的颜色向量
    Eigen::Vector3f point = payload.view_pos;//shading point位置点坐标
    Eigen::Vector3f normal = payload.normal;//shading point处的法向量

    Eigen::Vector3f result_color = {0, 0, 0};//作为返回的着色结果颜色向量
    for (auto& light : lights)//遍历光源
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f l = light.position - point;//view_position -> light source，入射光线方向向量l
        Eigen::Vector3f v = eye_pos - point; // view_position -> eye，观察向量v
        
        float r2 = l.dot(l);//r即是向量l的长度，根据长度公式和点乘计算公式，知：向量点乘自身，结果即是长度的平方

        //向量归一化
        l = l.normalized();
        v = v.normalized();

        //ka, kd, ks课程中介绍是数值常量, 此处被定义为Vector3f类型，使用cwiseProduct函数来实现数乘向量结果
        // ambient La=ka*Ia 
        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);

        // diffuse Ld=kd(I/r^2)max(0, nl)
        Eigen::Vector3f Ld = kd.cwiseProduct(light.intensity / r2) * std::max(0.0f, normal.normalized().dot(l));

        // specular Ls=ks(I/r^2)max(0, nh)^p
        Eigen::Vector3f h = ( l + v ).normalized();//半程向量归一化
        Eigen::Vector3f Ls = ks.cwiseProduct(light.intensity / r2)*std::pow(std::max(0.0f, normal.normalized().dot(h)), p);
        
        result_color += (La + Ld + Ls);
    }
    return result_color * 255.f;
}

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        // without interpolation
        return_color=payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());
        // with interpolation
        return_color=payload.texture->getColorBilinear(payload.tex_coords.x(), payload.tex_coords.y());
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;//纹理颜色值作为kd
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f l = light.position - point; // view_position -> light source
        Eigen::Vector3f v = eye_pos - point; // view_position -> eye

        float r2 = l.dot(l); // l cannot be normalized before, because r2 is the distance

        // note: l, v should be normalized
        l = l.normalized();
        v = v.normalized();

        // note: ka, kd, ks are scalars in lecture, but they are vectors here, so you should use cwiseProduct
        // ambient La=ka*Ia
        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity); // cwiseProduct--dot product

        // diffuse Ld=kd(I/r^2)max(0, nl)
        Eigen::Vector3f Ld = kd.cwiseProduct(light.intensity / r2) * std::max(0.0f, normal.normalized().dot(l));

        // specular Ls=ks(I/r^2)max(0, nh)^p
        Eigen::Vector3f h = (l + v).normalized();
        Eigen::Vector3f Ls = ks.cwiseProduct(light.intensity / r2) * std::pow(std::max(0.0f, normal.normalized().dot(h)), p);
        
        result_color += (La + Ld + Ls);
    }

    return result_color * 255.f;
}

Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here

    // Let n = normal = (x, y, z)
    auto x = normal.x(), y = normal.y(), z = normal.z();
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    Eigen::Vector3f t(x * y /std::sqrt(x * x + z * z), std::sqrt( x * x + z * z), z * y / sqrt( x * x + z * z));
    // Vector b = n cross product t
    Eigen::Vector3f b = normal.cross(t);
    // Matrix TBN = [t b n]
    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(),
           t.y(), b.y(), normal.y(),
           t.z(), b.z(), normal.z();//初始化顺序为行优先(存储后读取时为列优先)
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    float h = payload.texture->height, w=payload.texture->width, u=payload.tex_coords.x(), v=payload.tex_coords.y();
    float dU = kh * kn * (payload.texture->getColorBilinear( u + 1 / w, v).norm() - payload.texture->getColorBilinear(u, v).norm());//必须归一化
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    float dV = kh * kn * (payload.texture->getColorBilinear(u, v + 1 / h).norm()-payload.texture->getColorBilinear(u, v).norm());
    // Vector ln = (-dU, -dV, 1)
    Eigen::Vector3f ln(-dU, -dV, 1);
    // Normal n = normalize(TBN * ln)
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal;

    return result_color * 255.f;
}

Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here

    // Let n = normal = (x, y, z)
    auto x = normal.x(), y = normal.y(), z = normal.z();
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    Eigen::Vector3f t(x * y / std::sqrt(x * x + z * z), std::sqrt(x * x + z * z), z * y / sqrt(x * x + z * z));
    // Vector b = n cross product t
    Eigen::Vector3f b = normal.cross(t);
    // Matrix TBN = [t b n]
    Eigen::Matrix3f TBN;
    TBN << t.x(),b.x(),normal.x(),
           t.y(),b.y(),normal.y(),
           t.z(),b.z(),normal.z();
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    float h = payload.texture->height, w = payload.texture->width, u = payload.tex_coords.x(), v = payload.tex_coords.y();
    float dU= kh * kn * (payload.texture->getColorBilinear(u + 1 / w, v).norm()-payload.texture->getColorBilinear(u, v).norm());//归一化取值
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    float dV=kh * kn * (payload.texture->getColorBilinear(u, v + 1 / h).norm()-payload.texture->getColorBilinear(u, v).norm());
    // Vector ln = (-dU, -dV, 1)
    Eigen::Vector3f ln(-dU, -dV, 1);
    // Position p = p + kn * n * h(u,v)
    point += kn * normal * payload.texture->getColorBilinear(u, v).norm();
    // Normal n = normalize(TBN * ln)
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)//遍历光源
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f l = light.position - point; // view_position -> light source
        Eigen::Vector3f v = eye_pos - point; // view_position -> eye

        float r2 = l.dot(l); // l cannot be normalized before, because r2 is the distance

        // note: l, v should be normalized
        l = l.normalized();
        v = v.normalized();

        // note: ka, kd, ks are scalars in lecture, but they are vectors here, so you should use cwiseProduct
        // ambient La=ka*Ia
        Eigen::Vector3f La=ka.cwiseProduct(amb_light_intensity); // cwiseProduct--dot product

        // diffuse Ld=kd(I/r^2)max(0, nl)
        Eigen::Vector3f Ld=kd.cwiseProduct(light.intensity / r2)*std::max(0.0f, normal.normalized().dot(l));

        // specular Ls=ks(I/r^2)max(0, nh)^p
        Eigen::Vector3f h=(l+v).normalized();
        Eigen::Vector3f Ls=ks.cwiseProduct(light.intensity / r2)*std::pow(std::max(0.0f, normal.normalized().dot(h)), p);
        
        result_color+=(La+Ld+Ls);

    }

    return result_color * 255.f;
}


int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "../models/spot/hmap.jpg";
    r.set_texture(Texture(texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = displacement_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "../models/spot/spot_texture.png";
            r.set_texture(Texture(texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
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
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}