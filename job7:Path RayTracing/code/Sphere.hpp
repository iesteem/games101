#ifndef RAYTRACING_SPHERE_H
#define RAYTRACING_SPHERE_H
#include "Object.hpp"
#include "Vector.hpp"
#include "Bounds3.hpp"
#include "Material.hpp"

class Sphere : public Object{
public:
    Vector3f center;//球心坐标
    float radius, radius2;//半径、半径平方
    Material *m;//材质类型
    float area;//球体表面积

    Sphere(const Vector3f &c, const float &r, Material* mt = new Material()) : center(c), radius(r), radius2(r * r), m(mt), area(4 * M_PI *r *r) {}
    
    /*
    **光线ray与球体是否相交
    */
    bool intersect(const Ray& ray) {
        Vector3f L = ray.origin - center;
        float a = dotProduct(ray.direction, ray.direction);
        float b = 2 * dotProduct(ray.direction, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
        float area = 4 * M_PI * radius2;
        if (!solveQuadratic(a, b, c, t0, t1)) return false;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        return true;
    }

    /*
    **光线ray与球体是否相交，并记录多个交点中光线传播时间最小的情况tnear
    */
    bool intersect(const Ray& ray, float &tnear, uint32_t &index) const
    {

        Vector3f L = ray.origin - center;
        float a = dotProduct(ray.direction, ray.direction);
        float b = 2 * dotProduct(ray.direction, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1)) return false;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return false;
        tnear = t0;
        return true;
    }

    /*
    **光线ray与球体是否相交，并记录相交点数据
    */
    Intersection getIntersection(Ray ray){
        Intersection result;
        result.happened = false;
        Vector3f L = ray.origin - center;
        float a = dotProduct(ray.direction, ray.direction);
        float b = 2 * dotProduct(ray.direction, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1)) return result;
        if (t0 < 0) t0 = t1;
        if (t0 < 0) return result;
        result.happened=true;//t0作为选定的相交点时间开销(比t1花费时间更短)

        result.coords = Vector3f(ray.origin + ray.direction * t0);//相交点坐标
        result.normal = normalize(Vector3f(result.coords - center));//相交点法向量方向
        result.m = this->m;//材质类型
        result.obj = this;//物体类型
        result.distance = t0;//相交点光线传播时间
        return result;

    }


    void getSurfaceProperties(const Vector3f &P, const Vector3f &I, const uint32_t &index, const Vector2f &uv, Vector3f &N, Vector2f &st) const
    { N = normalize(P - center); }
    Vector3f evalDiffuseColor(const Vector2f &st)const {
        //return m->getColor();
    }

    //包围球体的AABB包围盒
    Bounds3 getBounds(){
        return Bounds3(Vector3f(center.x - radius, center.y - radius, center.z - radius),
                       Vector3f(center.x + radius, center.y + radius, center.z + radius));
    }

    /*
    **球体的采样(采用球坐标系)
    */
    void Sample(Intersection &pos, float &pdf){
        //球坐标系(r, theta, phi)
        //theta表示原点到点P的连线与+z轴之间的天顶角，phi表示原点到点P的连线在xy平面的投影线，与+x轴之间的方位角
        //对于整个球体，theta取值在[0,π]间，phi取值在[0,2π]间
        float theta = 2.0 * M_PI * get_random_float(), phi = M_PI * get_random_float();
        float theta = M_PI * get_random_float(), phi = 2.0 * M_PI * get_random_float();

        //理论上：将球坐标系(r, theta, phi)转化为直角坐标系(x, y, z)
        //理论上：x = r * sin(theta) * cos(phi), y = r * sin(theta) * sin(phi), z = r * cos(theta) 
        //此处视球坐标系中r取1，即单位球体

        //但此处：按(z, x, y)的方式建立直角坐标系取值
        Vector3f dir(std::cos(theta), std::sin(theta)*std::cos(phi), std::sin(theta)*std::sin(phi));//表示方向的单位向量        

        pos.coords = center + radius * dir;//光线起始点在球(直角)坐标系原点，方向即为原点到点P的连线的单位向量，求得相交点坐标
        pos.normal = dir;//法向量
        pos.emit = m->getEmission();//自发光
        pdf = 1.0f / area;//对球体表面积均匀采样，1 / 4πr*r
    }
    
    float getArea(){
        return area;
    }
    bool hasEmit(){
        return m->hasEmission();
    }
};




#endif //RAYTRACING_SPHERE_H
