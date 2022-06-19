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
**
*/
class BVHAccel {
public:
    enum class SplitMethod { NAIVE, SAH };
    BVHAccel(std::vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
    Bounds3 WorldBound() const;
    ~BVHAccel();

    Intersection Intersect(const Ray &ray) const;
    Intersection getIntersection(BVHBuildNode* node, const Ray& ray)const;
    bool IntersectP(const Ray &ray) const;
    BVHBuildNode* root;//BVH树根节点

    //构建BVH树
    BVHBuildNode* recursiveBuild(std::vector<Object*>objects);

    const int maxPrimsInNode;//
    const SplitMethod splitMethod;//
    std::vector<Object*> primitives;//
};

/*
**Bounding Volume Hierarchies的树节点
*/
struct BVHBuildNode {
    Bounds3 bounds;//存储包围盒(每个节点对应一个包围盒)
    BVHBuildNode *left;//左孩子
    BVHBuildNode *right;//右孩子
    Object* object;//包围盒内部物体类型
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
