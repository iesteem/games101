#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType { DIFFUSE};

class Material{
private:

    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    /*
    **半球坐标从局部坐标系转化为世界坐标系
    **输入形参：a表示半球坐标(局部坐标系)，
    */
    Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

public:
    MaterialType m_type;//材质类型
    Vector3f m_emission;//自发光
    float ior;//材质折射率
    Vector3f Kd, Ks;//diffuse、spacular
    float specularExponent;//specular函数的幂指数

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));//设置构造函数的默认参数
    inline MaterialType getType();
    inline Vector3f getColorAt(double u, double v);//根据uv坐标从纹理中获取颜色
    inline Vector3f getEmission();
    inline bool hasEmission();//判断是否有自发光

    //光线击中某点后，后续弹射的方向
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    //光线的pdf(概率密度函数probability density function，描述连续随机变量的概率分布)
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    //光线的贡献
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    m_emission = e;
}

MaterialType Material::getType(){return m_type;}
Vector3f Material::getEmission() {return m_emission;}

//这是一个空函数
Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}

//判断是否有自发光
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}


/*
**给定光线入射方向wi和法向量N，用XX分布采样出某个反射方向
*/
Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            float x_1 = get_random_float(), x_2 = get_random_float();//两个[0,1]的随机数
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z);
            float phi = 2 * M_PI * x_2;//对于上半球体，总立体角为2π，此时根据随机数x_2，随机选定立体角来确定反射光线的方向

            //上半球面上反射光线的方向
            //着色点在上半球面上，phi表示“着色点与球心的半径线”与+x轴所成夹角(逆时针)
            //x = r * cos(phi), y = r * sin(phi), z保持不变
            Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
            return toWorld(localRay, N);
            
            break;
        }
    }
}

/*
**概率密度函数
** 均匀采样概率 1 / (2 * PI)
**输入参数：wi入射光线方向，wo反射光线方向，N着色点处的法向量方向
*/
float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){//选择材质
        case DIFFUSE:
        {
            if (dotProduct(wo, N) > 0.0f)//反射光线与法向量夹角不超过90度
                return 0.5f / M_PI;// 1 / 2π
            else
                return 0.0f;
            break;
        }
    }
}


/*
**计算漫反射模型的贡献
*/
Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            float cosalpha = dotProduct(N, wo);//计算反射光线与法向量夹角的cos值
            if (cosalpha > 0.0f) {//夹角不超过90度
                Vector3f diffuse = Kd / M_PI;//为什么是kd/π？
                return diffuse;
            }
            else
                return Vector3f(0.0f);
            break;
        }
    }
}

#endif //RAYTRACING_MATERIAL_H
