#include "pathtracer.hpp"
#include "mat.hpp"
#include <opencv2/opencv.hpp>
#include <omp.h>

PathTracer::PathTracer(int width, int height, int spp)
    : width_(width), height_(height), spp_(spp), framebuffer_(width * height) {}

void PathTracer::buildBVH(const Scene &scene) {
    lights_.clear();
    for (const auto light : scene.lights) { // temporary solution for point lights
        Triangle lightTri1;
        lightTri1.v0 = light.position + Eigen::Vector3f(1.0f, 0.0f, 1.0f);
        lightTri1.v1 = light.position + Eigen::Vector3f(-1.0f, 0.0f, 1.0f);
        lightTri1.v2 = light.position + Eigen::Vector3f(1.0f, 0.0f, -1.0f);
        lightTri1.n0 = lightTri1.n1 = lightTri1.n2 = Eigen::Vector3f(0.0f, -1.0f, 0.0f);
        lights_.push_back(lightTri1);
    }

    std::vector<Triangle> triangles;
    for(const auto &object : scene.objects) {
        for (const auto &face : object.faces) {
            auto vertices = face->getVertices();
            Triangle tri;
            tri.v0 = vertices[0]->position;
            tri.v1 = vertices[1]->position;
            tri.v2 = vertices[2]->position;
            tri.n0 = vertices[0]->computeNormal();
            tri.n1 = vertices[1]->computeNormal();
            tri.n2 = vertices[2]->computeNormal();
            tri.t0 = vertices[0]->textureCoordinates;
            tri.t1 = vertices[1]->textureCoordinates;
            tri.t2 = vertices[2]->textureCoordinates;
            tri.area = ((tri.v1 - tri.v0).cross(tri.v2 - tri.v0)).norm() * 0.5f;
            tri.material = std::make_shared<Material>(object.material);
            Lambertian triMat(object.material);
            tri.mat = std::make_shared<Lambertian>(triMat);
            triangles.push_back(tri);
        }
    }
    std::cout << "Triangles loaded" << std::endl;
    bvh_ = BVHAccel(triangles);
    std::cout << "BVH finished" << std::endl;
}

Eigen::Vector3f PathTracer::estimateDirect(const HitRecord &hit, const Eigen::Vector3f &wo) {
    if (lights_.empty())
        return Eigen::Vector3f::Zero();

    int rndIndex = int(randomFloat() * lights_.size());
    const Triangle &light = lights_[rndIndex];

    float pdf;
    Eigen::Vector3f lightPoint = light.sample(pdf);
    Eigen::Vector3f wi = (lightPoint - hit.position).normalized();
    float distanceSquared = (lightPoint - hit.position).squaredNorm();
    
    Ray shadowRay(hit.position + wi * EPSILON, wi);
    HitRecord shadowHit;
    shadowHit.t = std::sqrt(distanceSquared) - EPSILON;
    if (bvh_.intersect(shadowRay, shadowHit))
        return Eigen::Vector3f::Zero(); // In shadow

    float cosTheta = std::max(0.0f, hit.normal.dot(wi));
    float cosLight = std::max(0.0f, shadowHit.normal.dot(-wi));
    Eigen::Vector3f brdf = hit.mat->evaluate(hit, wo, wi);
    pdf = pdf * (1.0f / lights_.size());
    Eigen::Vector3f emission = {100.0f, 100.0f, 100.0f}; // Assuming a constant emission for now

    return brdf.cwiseProduct(emission) * (cosTheta * cosLight) / (distanceSquared * pdf);
}

Eigen::Vector3f PathTracer::trace(const Ray &ray) {
    HitRecord hit;
    if (!bvh_.intersect(ray, hit))
        return Eigen::Vector3f(1.0f, 0.0f, 0.0f); // Background colour

    Eigen::Vector3f wo = -ray.direction;
    Eigen::Vector3f directLight = estimateDirect(hit, wo);

    Eigen::Vector3f wi;
    float pdf;
    Eigen::Vector3f brdf = hit.mat->sample(hit, wo, wi, pdf);
    float cosTheta = std::max(0.0f, hit.normal.dot(wi));
    if (pdf <= EPSILON)
        return directLight;

    float prr = std::min(0.1f, std::max(brdf.x(), std::max(brdf.y(), brdf.z())));
    if (randomFloat() > prr)
        return directLight; // Russian roulette termination
    
    Ray next(hit.position + wi * EPSILON, wi);
    Eigen::Vector3f indirectLight = trace(next);
    indirectLight = brdf.cwiseProduct(indirectLight) * cosTheta / pdf;

    return directLight + (indirectLight / prr);
}

Eigen::Vector3f PathTracer::samplePixel(int x, int y, const Camera &cam) {
    Eigen::Vector3f color(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < spp_; ++i) {
        float u = (x + randomFloat()) / width_;
        float v = (y + randomFloat()) / height_;
        Ray ray = cam.getRay(u, v);
        color += trace(ray);
    }
    return color / spp_;
}

void PathTracer::saveImage(const std::string &outputFile) {
    std::stringstream ss;
    ss << outputFile << ".ppm";
    const char* filename = ss.str().c_str();
    FILE* fp = fopen(filename, "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", width_, height_);
    for (auto i = 0; i < height_ * width_; i++) {
        static unsigned char colour[3];
        colour[0] = (unsigned char)(255 * std::pow(std::clamp(0.0f, 1.0f, framebuffer_[i].x()), 0.6f));
        colour[1] = (unsigned char)(255 * std::pow(std::clamp(0.0f, 1.0f, framebuffer_[i].y()), 0.6f));
        colour[2] = (unsigned char)(255 * std::pow(std::clamp(0.0f, 1.0f, framebuffer_[i].z()), 0.6f));
        fwrite(colour, 1, 3, fp);
    }
    fclose(fp);
}

void PathTracer::render(const Scene &scene, const Camera &camera, const std::string &outputFile) {
    std::cout << "Build BVH" << std::endl;
    buildBVH(scene);

    float progress = 0.0f;
    updateProgress(progress);
    #pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < height_; i++) {
        for (int j = 0; j < width_; j++) {
            framebuffer_[i * width_ + j] = samplePixel(j, i, camera);
        }
        progress += 1.0f / height_;
        updateProgress(progress);
    }
    updateProgress(1.0f);
    saveImage(outputFile);
}