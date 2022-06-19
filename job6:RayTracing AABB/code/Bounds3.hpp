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

    Vector3f Centroid() { return 0.5 * pMin + 0.5 * pMax; }//线段的质心坐标

    /*
    **如果两个包围盒存在重叠部分(重叠部分也是包围盒)，则能找到符合包围盒定义(pMin的各坐标值小于pMax的各坐标值)的pMin点和pMax点
    **如果两个包围盒没有重叠部分，则计算出来的pMin点和pMax点不满足包围盒定义
    */
    Bounds3 Intersect(const Bounds3& b)
    {
        return Bounds3(Vector3f(fmax(pMin.x, b.pMin.x), fmax(pMin.y, b.pMin.y),
                                fmax(pMin.z, b.pMin.z)),
                       Vector3f(fmin(pMax.x, b.pMax.x), fmin(pMax.y, b.pMax.y),
                                fmin(pMax.z, b.pMax.z)));
    }

    /*
    **选定点P与包围盒端点pMin的距离 占 包围盒端点pMin、pMax之间距离 的比例
    **用于计算插值
    */
    Vector3f Offset(const Vector3f& p) const
    {
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)//满足包围盒定义的情况
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    /*
    **判断两个包围盒是否部分重叠
    */
    bool Overlaps(const Bounds3& b1, const Bounds3& b2)
    {
        bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
        bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
        bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
        return (x && y && z);
    }

    /*
    **判断选定点p是否在在包围盒内
    */
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
**输入参数：ray作为光线信息，invDir和dirIsNeg用于优化计算过程
**invDir: ray direction(x,y,z), invDir=(1.0/x,1.0/y,1.0/z), use this because Multiply is faster that Division
**dirIsNeg: ray direction(x,y,z), dirIsNeg=[int(x>0),int(y>0),int(z>0)], use this to simplify your logic
*/
inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir, const std::array<int, 3>& dirIsNeg) const
{
    //存储光线进入和退出包围盒的时间
    float tEnter = FLT_MIN;
    float tExit  = FLT_MAX;

    for (int i=0; i<3; i++)//对于Vector3f类型，对x、y、z三条坐标轴各自计算 理论上进入和退出包围盒的时间
    {
        float t_min = (pMin[i] - ray.origin[i]) * invDir[i];//相交的最小时间
        float t_max= (pMax[i] - ray.origin[i]) * invDir[i];//相交的最大时间
        if (dirIsNeg[i]==0)    std::swap(t_min, t_max); // note: here must be ==0, because dirIsNeg is actually int(x>0)
        tEnter = std::max(t_min, tEnter);//tEnter取三条轴各自相交时间的最大值(只有三条轴全部进入才算进入包围盒)
        tExit = std::min(t_max, tExit);//tExit取三条轴各自相交时间的最小值(只要退出某条轴就算退出包围盒)
    }
    return tEnter<tExit && tExit>0;
}

/*
**合并两个包围盒，以得到更大的包围盒(新包围盒体积比前两者相加更大)
*/
inline Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    ret.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return ret;
}

/*
**尝试扩展包围盒，以将选定点p纳入新包围盒内
**如果点p在包围盒b外，扩充包围盒
**如果点p在包围盒b内，包围盒不变
*/
inline Bounds3 Union(const Bounds3& b, const Vector3f& p)
{
    Bounds3 ret;
    ret.pMin = Vector3f::Min(b.pMin, p);
    ret.pMax = Vector3f::Max(b.pMax, p);
    return ret;
}

#endif // RAYTRACING_BOUNDS3_H
