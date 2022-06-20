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

class BVHAccel {
public:
    enum class SplitMethod { NAIVE, SAH };//切分方法

    BVHAccel(std::vector<Object*> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE);
    Bounds3 WorldBound() const;
    ~BVHAccel();

    Intersection Intersect(const Ray &ray) const;
    Intersection getIntersection(BVHBuildNode* node, const Ray& ray)const;
    bool IntersectP(const Ray &ray) const;
    BVHBuildNode* root;//BVH树根节点(可理解为：BVH树)

    //传入所有objects，返回建立的BVH树根节点
    BVHBuildNode* recursiveBuild(std::vector<Object*>objects);

    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    std::vector<Object*> primitives;

    void getSample(BVHBuildNode* node, float p, Intersection &pos, float &pdf);
    void Sample(Intersection &pos, float &pdf);
};

struct BVHBuildNode {
    Bounds3 bounds;
    BVHBuildNode *left;
    BVHBuildNode *right;
    Object* object;
    float area;//BVH树的节点内包围盒面积

public:
    int splitAxis=0, firstPrimOffset=0, nPrimitives=0;
    BVHBuildNode(){
        bounds = Bounds3();
        left = nullptr;right = nullptr;
        object = nullptr;
    }
};


#endif //RAYTRACING_BVH_H
