#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"
class Object;
class Sphere;

//相交结构，用于村粗光线与物体相交时的各种数据
struct Intersection
{
    Intersection(){
        happened=false;
        coords=Vector3f();
        normal=Vector3f();
        distance= std::numeric_limits<double>::max();
        obj =nullptr;
        m=nullptr;
    }
    bool happened;
    Vector3f coords;//相交点坐标
    Vector3f normal;//相交点法向量
    double distance;//光线方程中的t值
    Object* obj;
    Material* m;
};
#endif //RAYTRACING_INTERSECTION_H
