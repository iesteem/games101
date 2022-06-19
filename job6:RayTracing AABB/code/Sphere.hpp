#ifndef RAYTRACING_SPHERE_H
#define RAYTRACING_SPHERE_H
#include "Object.hpp"
#include "Vector.hpp"
#include "Bounds3.hpp"
#include "Material.hpp"

class Sphere : public Object{
public:
    Vector3f center;
    float radius, radius2;
    Material *m;//
    Sphere(const Vector3f &c, const float &r) : center(c), radius(r), radius2(r * r), m(new Material()) {}

    /*
    **判断光线与球是否相交，同时保存相交点的着色信息
    */
    Intersection getIntersection(Ray ray)
    {
        Intersection result;//作为返回结果
        result.happened = false;//默认未相交

        //判断圆是否与光线相交
        Vector3f L = ray.origin - center;
        float a = dotProduct(ray.direction, ray.direction);
        float b = 2 * dotProduct(ray.direction, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1)) return result;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return result;
        result.happened=true;

        result.coords = Vector3f(ray.origin + ray.direction * t0);//根据光线方程求的相交点坐标
        result.normal = normalize(Vector3f(result.coords - center));//根据球心求得相交点法向量方向(方向即意味着取归一化得到单位向量)
        result.m = this->m;
        result.obj = this;//this表示调用当前函数的Sphere对象
        result.distance = t0;
        return result;

    }

    /*
    **？？？
    */
    void getSurfaceProperties(const Vector3f &P, const Vector3f &I, const uint32_t &index, const Vector2f &uv, Vector3f &N, Vector2f &st) const
    { N = normalize(P - center); }

    /*
    **获取球的diffuse系数
    */
    Vector3f evalDiffuseColor(const Vector2f &st)const {
        return m->getColor();//diffuse系数需要通过Material对象来获取
    }
    
    /*
    **获取球的包围盒(通过二维平面更好理解)
    */
    Bounds3 getBounds(){
        return Bounds3(Vector3f(center.x-radius, center.y-radius, center.z-radius),
                       Vector3f(center.x+radius, center.y+radius, center.z+radius));
    }
};




#endif //RAYTRACING_SPHERE_H
