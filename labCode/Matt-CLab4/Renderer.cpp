//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include <sstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include "Material.hpp"
#ifdef _OPENMP
    #include <omp.h>
#endif


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

// const float EPSILON = 1e-2;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    // float imageAspectRatio = scene.width / (float)scene.height;
    float imageAspectRatio = scene.height / (float)scene.width;
    Vector3f eye_pos(0, -30, -2);

    std::cout << "SPP: " << scene.spp << "\n";

    float progress = 0.0f;

#pragma omp parallel for num_threads(8) // use multi-threading for speedup if openmp is available
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {

            int m = i + j * scene.width;
            if(scene.spp == 1){
                // Task 1.2: Pixel Projection
                // Compute the Normalized Device Coordinates (NDC) of the pixel
                float x = -(2 * (i + 0.5) / (float)scene.width - 1) * imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                // Compute the ray direction in world coordinates
                Vector3f dir = normalize(Vector3f(x, y, 1));

                // Cast the ray into the scene and store the color in the framebuffer
                framebuffer[m] = scene.castRay(Ray(eye_pos, dir), 0);
            } else {
                // Task 4: Multi-sampling
                Vector3f result_colour = Vector3f(0);
                for (int s = 0; s < scene.spp; s++) {
                    // Compute the random offsets for jittering
                    float x_jitter = get_random_float();
                    float y_jitter = get_random_float();

                    // Compute the Normalized Device Coordinates (NDC) of the pixel with jittering
                    float x = -(2 * (i + x_jitter) / (float)scene.width - 1) * scale;
                    float y = (1 - 2 * (j + y_jitter) / (float)scene.height) * imageAspectRatio * scale;

                    // Compute the ray direction in world coordinates
                    Vector3f dir = normalize(Vector3f(x, 1, y));

                    // Cast the ray into the scene and accumulate the color
                    if (TASK_N >= 6) {
                        result_colour += scene.traceRay(Ray(eye_pos, dir), 0);
                        // result_colour += scene.castRay(Ray(eye_pos, dir), 0);
                    }
                    else
                        result_colour += scene.castRay(Ray(eye_pos, dir), 0);
                }
                // Average the color over the number of samples
                result_colour = result_colour / scene.spp;

                // Store the color in the framebuffer
                framebuffer[m] = result_colour;
            }
        }
        progress += 1.0f / (float)scene.height;
        UpdateProgress(progress);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    std::stringstream ss;
    ss << "binary_task" << TASK_N<<".ppm";
    std::string str = ss.str();
    const char* file_name = str.c_str();
    FILE* fp = fopen(file_name, "wb");
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
