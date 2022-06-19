#ifndef RAYTRACING_RAY_H
#define RAYTRACING_RAY_H
#include "Vector.hpp"
struct Ray{
    //Destination = origin + t*direction
    Vector3f origin;//光线的初始位置点
    Vector3f direction;//光线的前进方向(单位向量)
    Vector3f direction_inv;//direction的各坐标倒数组成的向量
    double t;//光线传播时间，用以确定光线与其他物体的相交点坐标
    double t_min, t_max;

    Ray(const Vector3f& ori, const Vector3f& dir, const double _t = 0.0): origin(ori), direction(dir),t(_t) {
        direction_inv = Vector3f(1./ direction.x, 1./ direction.y, 1./ direction.z);
        t_min = 0.0;
        t_max = std::numeric_limits<double>::max();

    }

    Vector3f operator()(double t) const{return origin + direction * t;}

    friend std::ostream &operator<<(std::ostream& os, const Ray& r){
        os<<"[origin:="<<r.origin<<", direction="<<r.direction<<", time="<< r.t<<"]\n";
        return os;
    }
};
#endif //RAYTRACING_RAY_H
