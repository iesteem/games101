#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H
#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include "Object.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

struct BVHBuildNode;
struct BVHPrimitiveInfo;

inline int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;

/*
**BVHAccel类型封装了对BVH树操作的数据结构
*/
class BVHAccel {
public:
    enum class SplitMethod { NAIVE, SAH };//切分方法
    const int maxPrimsInNode;//节点内的最大物体个数
    const SplitMethod splitMethod;//枚举类数据成员
    std::vector<Object*> primitives;//容纳所有object的vector
    BVHBuildNode* root;//BVH树根节点(可理解为：BVH树)

    BVHAccel(std::vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
    Bounds3 WorldBound() const;
    ~BVHAccel();

    Intersection Intersect(const Ray &ray) const;
    Intersection getIntersection(BVHBuildNode* node, const Ray& ray)const;
    bool IntersectP(const Ray &ray) const;
    

    //传入所有objects，返回建立的BVH树根节点
    BVHBuildNode* recursiveBuild(std::vector<Object*>objects);

    void getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf);
    void Sample(Intersection &pos, float &pdf);
};

/*
**Bounding Volume Hierarchies的树节点
*/
struct BVHBuildNode {
    Bounds3 bounds;//存储包围盒(每个节点对应一个包围盒)
    BVHBuildNode *left;//左孩子
    BVHBuildNode *right;//右孩子
    Object* object;//包围盒内部物体类型
    float area;//BVH树的节点内包围盒面积
public:
    int splitAxis=0, firstPrimOffset=0, nPrimitives=0;

    //无参构造函数
    BVHBuildNode(){
        bounds = Bounds3();
        left = nullptr;right = nullptr;
        object = nullptr;
    }
};

#endif //RAYTRACING_BVH_H
