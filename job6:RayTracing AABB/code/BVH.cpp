#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode, SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod), primitives(std::move(p))
{
    time_t start, stop;//开始时间、停止时间

    time(&start);//start被给定为当前时间
    if (primitives.empty())
        return;
    root = recursiveBuild(primitives);
    time(&stop);//stop被给定为当前时间

    double diff = difftime(stop, start);//计算耗费的时间
    //按时、分、秒形式给出消耗的时间
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

/*
**构建BVH树
*/
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();//无参构建根节点

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;//无参构造包围盒bounds
    for (int i = 0; i < objects.size(); ++i)//遍历objects中的所有object
    {
        bounds = Union(bounds, objects[i]->getBounds());//更新bounds，以容纳所有objects的包围盒
    }
        
    if (objects.size() == 1) {//objects中的object总数为1
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();//将唯一物体的包围盒赋值给node的bounds数据成员
        node->object = objects[0];//将唯一物体给node的object数据成员
        node->left = nullptr;//无孩子节点
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {//objects中的object总数为2
        //根节点类似为空，唯二的物体作为根节点的左右孩子
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);//更新更节点对应的包围盒(根节点的object数据成员为nullptr)
        return node;
    }
    else {//objects中的object总数大于2
        Bounds3 centroidBounds;//无参构建包围盒
        for (int i = 0; i < objects.size(); ++i)//遍历objects中的所有object
        {
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());//更新centroidBounds，以容纳所有object的包围盒
        }
            
        int dim = centroidBounds.maxExtent();//包围盒的最大边
        switch (dim) {
        case 0://x边最大
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1://y边最大
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2://z边最大
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);//存储左子树的全部objects
        auto rightshapes = std::vector<Object*>(middling, ending);//存储右子树的全部objects

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);//递归构建左子树
        node->right = recursiveBuild(rightshapes);//递归构建右子树

        node->bounds = Union(node->left->bounds, node->right->bounds);//更新node->bounds，以容纳左、右子树的全部包围盒
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;//存储相交数据
    if (!root)
       return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

/*
**判断光线与BVH树是否相交，返回相交数据(场景中可能存在多个BVH树)
*/
Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    Intersection inter;//存储相交数据
    Vector3f indiv(1.0f/ray.direction[0], 1.0f/ray.direction[1], 1.0f/ray.direction[2]);//用乘法代替除法以加速运算：此为光线的法向量各坐标值的倒数组成的向量，用于除以法向量的情况
    std::array<int, 3> dirIsNeg;
    dirIsNeg[0] = int(ray.direction.x > 0);//这是坐标问题。世界坐标系？摄像机位置？相交点坐标为正为负？？？
    dirIsNeg[1] = int(ray.direction.y > 0);
    dirIsNeg[2] = int(ray.direction.z > 0);

    //如果给定光线与BVH树node所存储的包围盒不相交，退出函数
    if (!node->bounds.IntersectP(ray, indiv, dirIsNeg))    return inter;

    //如果给定光线与BVH树node所存储的包围盒相交，且该node是叶子节点
    if (node->left==nullptr && node->right==nullptr)
    {
        inter=node->object->getIntersection(ray);//判断光线与该物体是否相交，并存储相交数据(要去看各object类型的实现函数)
        return inter;
    }
    
    //与包围盒相交，但node不是叶子节点时，要递归判断光线与node的左右子树是否相交
    Intersection left = getIntersection(node->left, ray);
    Intersection right = getIntersection(node->right, ray);
    return left.distance<right.distance ? left : right;//返回距离最近的相交点的相交数据(未考虑光线的递归反射、递归折射)
}