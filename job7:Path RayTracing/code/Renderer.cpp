#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

//This where we iterate over all pixels in the image,
//generate primary rays and cast these rays into the scene. The content of the
//framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);//存储像素点颜色的数组

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // int spp = 16;
    int spp = 64;//每个像素64个采样点
    std::cout << "SPP: " << spp << "\n";
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {//遍历所有像素点
            float x = (2 * (i + 0.5) / (float)scene.width - 1) * imageAspectRatio * scale;//x怎么算的？
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            Vector3f dir = normalize(Vector3f(-x, y, 1));//光线方向的世界坐标？
            for (int k = 0; k < spp; k++){//在像素点内循环spp次
                framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;//插值全部采样数据  
            }
            m++;
        }
        UpdateProgress(j / (float)scene.height);//？？？
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
