#pragma once
#include <cstring>
#include "Object.hpp"


bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, const Vector3f& orig,
                          const Vector3f& dir, float& tnear, float& u, float& v)
{
    // TODO: Implement this function that tests whether the triangle
    // that's specified bt v0, v1 and v2 intersects with the ray (whose
    // origin is *orig* and direction is *dir*)
    // Also don't forget to update tnear, u and v.
    //克莱姆法则
    Vector3f E1 = v1 - v0;
    Vector3f E2 = v2 - v0;
    Vector3f S = orig - v0;
    Vector3f S1 = crossProduct(dir, E2);
    Vector3f S2 = crossProduct(S, E1);

    tnear = 1.0f / dotProduct(S1, E1) * dotProduct(S2, E2);
    u = 1.0f / dotProduct(S1, E1) * dotProduct(S1, S);
    v = 1.0f / dotProduct(S1, E1) * dotProduct(S2, dir);

    // if the answer is valid, return true;
    if(tnear > 0.0f && v >= 0.0f && v <= 1.0f && u >= 0.0f && u <= 1.0f)  return true;
    else    return false;
}

class MeshTriangle : public Object
{
public:
    MeshTriangle(const Vector3f* verts, const uint32_t* vertsIndex, const uint32_t& numTris, const Vector2f* st)//numTris表示三角形个数
    {
        uint32_t maxIndex = 0;
        for (uint32_t i = 0; i < numTris * 3; ++i)//numTris个三角形，理论上有numTris * 3个顶点
        {
            if (vertsIndex[i] > maxIndex) maxIndex = vertsIndex[i];
        }
           
        maxIndex += 1;//numTris个三角形中实际存在的顶点个数

        //根据实际存在的顶点个数为数据成员赋值
        vertices = std::unique_ptr<Vector3f[]>(new Vector3f[maxIndex]);//所指涉的内部资源为“内含maxIndex个元素的Vector3f数组”
        memcpy(vertices.get(), verts, sizeof(Vector3f) * maxIndex);//复制到verts中
        vertexIndex = std::unique_ptr<uint32_t[]>(new uint32_t[numTris * 3]);//所指涉的内部资源为“内含numTris * 3个元素的uint32_t数组”
        memcpy(vertexIndex.get(), vertsIndex, sizeof(uint32_t) * numTris * 3);//复制到vertsIndex中
        numTriangles = numTris;//三角形个数
        stCoordinates = std::unique_ptr<Vector2f[]>(new Vector2f[maxIndex]);//所指涉的内部资源为“内含maxIndex个元素的Vector2f数组”
        memcpy(stCoordinates.get(), st, sizeof(Vector2f) * maxIndex);//复制到st中
    }

    /*
    **光线与MeshTriangle(内含多个三角形)相交
    */
    bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t& index, Vector2f& uv) const override
    {
        bool intersect = false;
        for (uint32_t k = 0; k < numTriangles; ++k)//遍历每个三角形
        {
            //确定三角形的三个顶点坐标
            const Vector3f& v0 = vertices[vertexIndex[k * 3]];
            const Vector3f& v1 = vertices[vertexIndex[k * 3 + 1]];
            const Vector3f& v2 = vertices[vertexIndex[k * 3 + 2]];
            float t, u, v;

            //选择最近的交点(光线与不同的三角形相交时)
            if (rayTriangleIntersect(v0, v1, v2, orig, dir, t, u, v) && t < tnear)//判断对当前三角形是否相交，同时判断当前t值是否小于当前tnear值
            {
                tnear = t;//更新tnear值
                uv.x = u;//
                uv.y = v;
                index = k;//更新三角形下标
                intersect |= true;
            }
        }

        return intersect;
    }

    void getSurfaceProperties(const Vector3f&, const Vector3f&, const uint32_t& index, const Vector2f& uv, Vector3f& N,
                              Vector2f& st) const override
    {
        //确定当前序号对应的三角形所包含的顶点坐标
        const Vector3f& v0 = vertices[vertexIndex[index * 3]];
        const Vector3f& v1 = vertices[vertexIndex[index * 3 + 1]];
        const Vector3f& v2 = vertices[vertexIndex[index * 3 + 2]];

        Vector3f e0 = normalize(v1 - v0);
        Vector3f e1 = normalize(v2 - v1);
        N = normalize(crossProduct(e0, e1));//三角形所在平面的单位法向量

        //这是纹理坐标吗？
        const Vector2f& st0 = stCoordinates[vertexIndex[index * 3]];
        const Vector2f& st1 = stCoordinates[vertexIndex[index * 3 + 1]];
        const Vector2f& st2 = stCoordinates[vertexIndex[index * 3 + 2]];

        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;//重心坐标
    }

    //？
    Vector3f evalDiffuseColor(const Vector2f& st) const override
    {
        float scale = 5;
        float pattern = (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
        return lerp(Vector3f(0.815, 0.235, 0.031), Vector3f(0.937, 0.937, 0.231), pattern);//插值
    }

    std::unique_ptr<Vector3f[]> vertices;
    uint32_t numTriangles;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;
};
