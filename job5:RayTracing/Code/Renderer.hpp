#pragma once
#include "Scene.hpp"

struct hit_payload
{
    float tNear;//确定最近交点的距离
    uint32_t index;//
    Vector2f uv;//
    Object* hit_obj;//指向物体的指针
};

class Renderer
{
public:
    void Render(const Scene& scene);

private:
};