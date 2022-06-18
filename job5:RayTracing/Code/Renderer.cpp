#include <fstream>
#include "Vector.hpp"
#include "Renderer.hpp"
#include "Scene.hpp"
#include <optional>

/*
**将角度转换为弧度
*/
inline float deg2rad(const float &deg)
{ return deg * M_PI/180.0; }

/*
**确定反射光线direction
*/
Vector3f reflect(const Vector3f &I, const Vector3f &N)
{
    return I - 2 * dotProduct(I, N) * N;
}


/*？？？？？？
******根据Snell's law计算折射光线direction
**考虑入射光线与物体的关系：1.光线在物体内部；2.光线在物体外部
**1.当光线在物体内部时，反转折射关系(对折射率取到数)，并对法向量取反
**2.当光线在物体外部时，
*/
// If the ray is outside, you need to make cosi positive cosi = -N.I
//
// If the ray is inside, you need to invert the refractive indices and negate the normal N
Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior)
{
    //如果光线在物体内部，I与N的夹角应大于90度，则cosi值为负；
    //如果光线在物体外部，I与N的夹角应小于90度，则cosi值为正；
    float cosi = clamp(-1, 1, dotProduct(I, N));//计算入射光线I与法向量N的点乘，对于单位向量，此结果即为夹角的cos值

    //ior表示折射率，即值为sin入射角/sin折射角。经过等价代换后，赋值
    float etai = 1, etat = ior;//etai表示入射光线所在介质的反射率n，etat表示折射光线所在介质的反射率nt

    //？？？？
    Vector3f n = N;
    if (cosi < 0)//光线在物体内部
    {
        cosi = -cosi;//根据cos(π-x) = -cosx
    }  
    else //光线在物体外部
    { 
        std::swap(etai, etat);
        n= -N; 
    }

    //见“知乎--孙小磊”
    ///etai表示入射光线所在介质的反射率，etat表示折射光线所在介质的反射率
    float eta = etai / etat;//eta值为 n/nt
    float k = 1 - eta * eta * (1 - cosi * cosi);//折射光线dirction计算公式中的第二项(根号下的全部值)
    return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;//当k小于0时，开方意义不存在，直接取0值
}


/*
**计算Fresnel Reflection(全反射)
**I为入射光线方向(单位向量)，N为交点处的法向量，ior为折射率
**返回反射率[0,1]
*/
float fresnel(const Vector3f &I, const Vector3f &N, const float &ior)
{
    //i表示入射角相关数据，t表示折射角相关数据
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;
    if (cosi > 0) {  std::swap(etai, etat); }
    // Compute sini using Snell's law

    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));//折射角的sin值,据此计算反射率

    if (sint >= 1) {//折射角大于90度，说明无折射，全反射
        return 1;
    }
    else//折射角小于90度，存在折射
    {
        float cost = sqrtf(std::max(0.f, 1 - sint * sint));//折射角的cos值
        cosi = fabsf(cosi);//入射角的cos值(取绝对值)

        //根据折射率ior知，n/nt = sin折射角/sin入射角 = 1 / ior
        //etai表示入射光线所在介质的反射率n，etat表示折射光线所在介质的反射率nt
        //不知道使用的哪个公式？？？
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        return (Rs * Rs + Rp * Rp) / 2;
    }
}

/*
**判断光线是否与物体相交
**形参：orig为光线起点；dir为光线方向(单位向量)；objects为场景内的所有物体组成的数组
**返回值：tnear表示光线与所有物体存在多个交点时，最近的交点距离；index表示三角形的序号；uv表示交点的
*/
std::optional<hit_payload> trace(const Vector3f &orig, const Vector3f &dir, const std::vector<std::unique_ptr<Object> > &objects)
{
    float tNear = kInfinity;
    std::optional<hit_payload> payload;
    for (const auto & object : objects)//遍历所有物体
    {
        float tNearK = kInfinity;//赋初值
        uint32_t indexK;
        Vector2f uvK;
        if (object->intersect(orig, dir, tNearK, indexK, uvK) && tNearK < tNear)//判断是否存在交点，并且交点距离是否最小
        {
            payload.emplace();
            payload->hit_obj = object.get();
            payload->tNear = tNearK;
            payload->index = indexK;
            payload->uv = uvK;
            tNear = tNearK;
        }
    }

    return payload;
}

// [comment]
// Implementation of the Whitted-style light transport algorithm (E [S*] (D|G) L)
//
// This function is the function that compute the color at the intersection point
// of a ray defined by a position and a direction. Note that thus function is recursive (it calls itself).
//
// If the material of the intersected object is either reflective or reflective and refractive,
// then we compute the reflection/refraction direction and cast two new rays into the scene
// by calling the castRay() function recursively. When the surface is transparent, we mix
// the reflection and refraction color using the result of the fresnel equations (it computes
// the amount of reflection and refraction depending on the surface normal, incident view direction
// and surface refractive index).
//
// If the surface is diffuse/glossy we use the Phong illumation model to compute the color
// at the intersection point.
// [/comment]
Vector3f castRay(const Vector3f &orig, const Vector3f &dir, const Scene& scene, int depth)
{
    if (depth > scene.maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }

    Vector3f hitColor = scene.backgroundColor;
    if (auto payload = trace(orig, dir, scene.get_objects()); payload)
    {
        Vector3f hitPoint = orig + dir * payload->tNear;
        Vector3f N; // normal
        Vector2f st; // st coordinates
        payload->hit_obj->getSurfaceProperties(hitPoint, dir, payload->index, payload->uv, N, st);
        switch (payload->hit_obj->materialType) {
            case REFLECTION_AND_REFRACTION:
            {
                Vector3f reflectionDirection = normalize(reflect(dir, N));
                Vector3f refractionDirection = normalize(refract(dir, N, payload->hit_obj->ior));
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint - N * scene.epsilon :
                                             hitPoint + N * scene.epsilon;
                Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
                                             hitPoint - N * scene.epsilon :
                                             hitPoint + N * scene.epsilon;
                Vector3f reflectionColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1);
                Vector3f refractionColor = castRay(refractionRayOrig, refractionDirection, scene, depth + 1);
                float kr = fresnel(dir, N, payload->hit_obj->ior);
                hitColor = reflectionColor * kr + refractionColor * (1 - kr);
                break;
            }
            case REFLECTION:
            {
                float kr = fresnel(dir, N, payload->hit_obj->ior);
                Vector3f reflectionDirection = reflect(dir, N);
                Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                             hitPoint + N * scene.epsilon :
                                             hitPoint - N * scene.epsilon;
                hitColor = castRay(reflectionRayOrig, reflectionDirection, scene, depth + 1) * kr;
                break;
            }
            default:
            {
                // [comment]
                // We use the Phong illumation model int the default case. The phong model
                // is composed of a diffuse and a specular reflection component.
                // [/comment]
                Vector3f lightAmt = 0, specularColor = 0;
                Vector3f shadowPointOrig = (dotProduct(dir, N) < 0) ?
                                           hitPoint + N * scene.epsilon :
                                           hitPoint - N * scene.epsilon;
                // [comment]
                // Loop over all lights in the scene and sum their contribution up
                // We also apply the lambert cosine law
                // [/comment]
                for (auto& light : scene.get_lights()) {
                    Vector3f lightDir = light->position - hitPoint;
                    // square of the distance between hitPoint and the light
                    float lightDistance2 = dotProduct(lightDir, lightDir);
                    lightDir = normalize(lightDir);
                    float LdotN = std::max(0.f, dotProduct(lightDir, N));
                    // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                    auto shadow_res = trace(shadowPointOrig, lightDir, scene.get_objects());
                    bool inShadow = shadow_res && (shadow_res->tNear * shadow_res->tNear < lightDistance2);

                    lightAmt += inShadow ? 0 : light->intensity * LdotN;
                    Vector3f reflectionDirection = reflect(-lightDir, N);

                    specularColor += powf(std::max(0.f, -dotProduct(reflectionDirection, dir)),
                        payload->hit_obj->specularExponent) * light->intensity;
                }

                hitColor = lightAmt * payload->hit_obj->evalDiffuseColor(st) * payload->hit_obj->Kd + specularColor * payload->hit_obj->Ks;
                break;
            }
        }
    }

    return hitColor;
}

// [comment]
// The main render function. This where we iterate over all pixels in the image, generate
// primary rays and cast these rays into the scene. The content of the framebuffer is
// saved to a file.
// [/comment]
/*
**遍历投影平面上的所有像素，用以生成投射光线来照向场景
*/
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);//缓存数组，数组大小为像素点总数

    float scale = std::tan(deg2rad(scene.fov * 0.5f));
    float imageAspectRatio = scene.width / (float)scene.height;//长宽比，用以得到x轴坐标值

    Vector3f eye_pos(0);//eye position
    int m = 0;
    for (int j = 0; j < scene.height; ++j)
    {
        for (int i = 0; i < scene.width; ++i)//遍历每个像素点
        {
            // TODO: Find the x and y positions of the current pixel to get the direction
            // vector that passes through it.

            // Also, don't forget to multiply both of them with the variable *scale*, and
            // x (horizontal) variable with the *imageAspectRatio* 

            // generate ray from each pixiel--(0,width-1)x(0,height-1)->(-1,1)x(-1,1)

            float x = ((i + 0.5f) * 2.0f / (float)scene.width - 1.0f) * scale * imageAspectRatio;
            float y = (1 - (j + 0.5f) * 2.0f / (float)scene.height) * scale;
            // note: the camera looks at -z axis, so z=-1 here
            Vector3f dir = Vector3f(x, y, -1); // Don't forget to normalize this direction!
           
            // actually we only need the direction of ray
            x = i + 0.5f - scene.width / 2.0f;
            y = scene.height / 2 - j - 0.5f;
            float z = -scene.height / 2;
            dir = Vector3f(x, y, z);
            
            dir = normalize(dir);
            
            framebuffer[m++] = castRay(eye_pos, dir, scene, 0);
        }
        UpdateProgress(j / (float)scene.height);
    }

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
