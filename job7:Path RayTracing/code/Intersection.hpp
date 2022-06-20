#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"
class Object;
class Sphere;

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
    bool happened;//是否相交
    Vector3f coords;//相交点坐标
    Vector3f tcoords;//相交点对应的纹理坐标
    Vector3f normal;//相交点法向量
    Vector3f emit;//自发光
    double distance;//相交点距光线起始点的传播时间
    Object* obj;//物体类型
    Material* m;//材质类型
};
#endif //RAYTRACING_INTERSECTION_H
