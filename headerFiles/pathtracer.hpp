#ifndef PATHTRACER_HPP
#define PATHTRACER_HPP

#include <vector>
#include <string>
#include <Eigen/Dense>
#include "camera.hpp"
#include "bvh.hpp"

/*by Matthew Reynolds u6949604*/
class PathTracer {
public:
    PathTracer(int width, int height, int spp);
    void render(const Scene &scene, const Camera &camera, const std::string &outputFile);

private:
    int width_;
    int height_;
    int spp_;
    std::vector<Eigen::Vector3f> framebuffer_;
    BVHAccel bvh_;
    std::vector<Triangle> lights_;

    void buildBVH(const Scene &scene);
    Eigen::Vector3f trace(const Ray &ray);
    Eigen::Vector3f samplePixel(int x, int y, const Camera &cam);
    void saveImage(const std::string &outputFile);
    Eigen::Vector3f estimateDirect(const HitRecord &hit, const Eigen::Vector3f &wo);
};

#endif // PATHTRACER_HPP