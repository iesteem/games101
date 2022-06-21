#ifndef RAYTRACING_BOUNDS3_H
#define RAYTRACING_BOUNDS3_H
#include "Ray.hpp"
#include "Vector.hpp"
#include <limits>
#include <array>
#include<float.h>

class Bounds3
{
  public:
    Vector3f pMin, pMax;//需要两个Vector3f位置点来确定AABB包围盒，分别确定x，y，z轴上的大、小值平面
    //无参构造函数
    Bounds3()
    {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);//x，y，z轴上的小值平面
        pMin = Vector3f(maxNum, maxNum, maxNum);//x，y，z轴上的大值平面
    }
    //给定单个坐标构建包围盒
    Bounds3(const Vector3f p) : pMin(p), pMax(p) {}
    //给定两个坐标构建包围盒
    Bounds3(const Vector3f p1, const Vector3f p2)
    {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    //点pMin指向点pMax的向量
    Vector3f Diagonal() const { return pMax - pMin; }

    //找到包围盒的最长边
    int maxExtent() const
    {
        Vector3f d = Diagonal();
        if (d.x > d.y && d.x > d.z) //0表示dx最大，即x边最大
            return 0;
        else if (d.y > d.z)         //1表示dy最大，即y边最大
            return 1;
        else                        //2表示dz最大，即z边最大
            return 2;
    }

    double SurfaceArea() const
    {
        Vector3f d = Diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }//包围盒的中心坐标
    
    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    Vector3f Offset(const Vector3f& p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    bool Inside(const Vector3f& p, const Bounds3& b)
    {
        return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
                p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
    }
    inline const Vector3f& operator[](int i) const
    {
        return (i == 0) ? pMin : pMax;
    }

    inline bool IntersectP(const Ray& ray, const Vector3f& invDir,
                           const std::array<int, 3>& dirisNeg) const;
};

/*
**
**invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
**dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
*/
inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{
    float tEnter = FLT_MIN;//因为tEnter要取多个t_min的最大值，因此初始值被设置为极小值
    float tExit = FLT_MAX;//因为tExit要取多个t_max的最小值，因此初始值被设置为极大值
    for (int i = 0; i < 3; i++)
    {
        float t_min = (pMin[i] - ray.origin[i]) * invDir[i];
        float t_max = (pMax[i] - ray.origin[i]) * invDir[i];
        if (dirIsNeg[i] == 0)    std::swap(t_min, t_max); // note: here must be ==0, because dirIsNeg is actually int(x>0)
        tEnter = std::max(t_min, tEnter);
        tExit = std::min(t_max, tExit);
    }
    return tEnter<=tExit && tExit>=0;
}


inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
