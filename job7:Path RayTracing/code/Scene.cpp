#include "Scene.hpp"

/*
**为scene创建BVH树
*/
void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
    //objects形参来源于Scene类型中的数据成员，其内包含了scene内的全部object
    //第二个形参 1 表示每个包围盒内仅包含一个物体
    //NAIVE指BVH中对物体的划分方法
}

/*
**判断光线是否与BVH树相交
*/
Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

/*
**
*/
void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {//遍历所有物体
        if (objects[k]->hasEmit()){//是否有自发光情况(光源)
            emit_area_sum += objects[k]->getArea();//将当前物体的采样面积加入总面积中
        }
    }
    float p = get_random_float() * emit_area_sum;//为什么要乘以随机数？
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {//遍历所有物体
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

/*
**找到与光线相交的物体，从中选择距离最近的相交点及被击中的物体
*/
bool Scene::trace(const Ray &ray, const std::vector<Object*> &objects, float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)//遍历所有物体
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;

        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {//当前object与光线相交，且相交点距离更近
            *hitObject = objects[k];//设置当前object作为hitobject
            tNear = tNearK;//更新tNear
            index = indexK;//更新index
        }
    }
    return (*hitObject != nullptr);//否则，所有物体均不与光线相交
}

/*
**Path Tracing路径追踪算法
*/
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Vector3f L_dir(0,0,0);
    Vector3f L_indir(0,0,0);

    Intersection inter = intersect(ray); //光线ray与BVH树的交点

    if(!inter.happened) return Vector3f(0,0,0);//未击中BVH树

    //前提：已击中BVH树
    if(inter.m->hasEmission())//如果存在自发光
    {
        // if(depth==0)    return inter.m->getEmission(); // if this ray hit light source directly, return directly.
        // else return Vector3f(0,0,0); // if thie ray hit light source(but not directly), we do not consider light source(we will consider it later)
        return inter.m->getEmission();//返回自发光：数据成员Vector3f m_emission
    }

    //对光源采样
    Intersection lightInter;
    float pdf_light = 0.0f;
    sampleLight(lightInter, pdf_light);

    Vector3f normal = inter.normal;//被击中物体的法向量
    Vector3f object2light = lightInter.coords-inter.coords;//向量，由BVH与光线的相交点指向光源
    float objectLight_distance = object2light.norm();//对object2light取向量距离的平方
    object2light = object2light.normalized();//object2light向量归一化

    Ray light(inter.coords, object2light);//构建光线light：光线起始点在”光线与BVH树的交点“，方向为指向光源

    Intersection object2lightInter = intersect(light);//发射light，找到其与BVH树的交点(有问题)

    // if light ray hit light source directly
    if(object2lightInter.happened && (object2lightInter.coords-lightInter.coords).norm()<0.1)
    {
        // L_dir = emit * eval (wo , ws , N) * dot (ws , N) * dot (ws ,NN) / |x-p |^2 / pdf_light
        L_dir = lightInter.emit*inter.m->eval(ray.direction, object2light, normal)*dotProduct(object2light, normal)*dotProduct(-object2light, lightInter.normal)/(objectLight_distance*objectLight_distance)/pdf_light;
    }


    // hit other object
    // RR--get_random_float will directly return a float in 0-1
    if(get_random_float() < RussianRoulette)
    {
        // construct out ray
        // from object, sample object-0>outside 
        Vector3f outDirection = inter.m->sample(ray.direction, normal).normalized();
        Ray outRay(inter.coords, outDirection);
        Intersection outRayInter = intersect(outRay);

        // if out ray hit something but not light source--indirectly
        if(outRayInter.happened && !outRayInter.m->hasEmission())
        {
            // L_indir = shade (q, wi) * eval (wo , wi , N) * dot (wi , N)/ pdf (wo , wi , N) / RussianRoulette
            L_indir = castRay(outRay, depth+1)*inter.m->eval(ray.direction, outDirection, normal)*dotProduct(outDirection, normal)/inter.m->pdf(ray.direction, outDirection, normal)/RussianRoulette;
            // note: when we recursively call this funtion, depth+=1
        }
    }
    return L_dir + L_indir;
}