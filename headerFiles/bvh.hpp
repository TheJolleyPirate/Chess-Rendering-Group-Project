#ifndef BVH_HPP
#define BVH_HPP

#include <vector>
#include <memory>
#include "ray.hpp"
#include "global.hpp"
#include "object.hpp"
#include "scene.hpp"
#include <Eigen/Dense>
#include <algorithm>

struct Mat;

/*by Matthew Reynolds u6949604*/
struct HitRecord {
    Eigen::Vector3f position;
    Eigen::Vector3f normal;
    Eigen::Vector2f textureCoords; 
    float t = std::numeric_limits<float>::infinity();
    std::shared_ptr<Mat> mat;
    std::shared_ptr<Material> material;
};

/*by Matthew Reynolds u6949604*/
struct Triangle {
    Eigen::Vector3f v0, v1, v2;
    Eigen::Vector3f n0, n1, n2;
    Eigen::Vector2f t0, t1, t2; 
    std::shared_ptr<Mat> mat;
    std::shared_ptr<Material> material;
    float area;

    bool intersect(const Ray &ray, float &tOut, float &u, float &v) const;
    Eigen::Vector3f sample(float &pdf) const;
};

/*by Matthew Reynolds u6949604*/
class BVHNode {
public:
    Eigen::Vector3f boundsMin, boundsMax;
    std::unique_ptr<BVHNode> left, right;
    std::vector<Triangle> triangles;
    bool isLeaf() const { return !left && !right; }
};

/*by Matthew Reynolds u6949604*/
class BVHAccel {
public:
    BVHAccel() = default;
    BVHAccel(std::vector<Triangle> &triangles) { build(triangles); }
    void build(std::vector<Triangle> &triangles);
    bool intersect(const Ray &ray, HitRecord &hit) const;

private:
    std::unique_ptr<BVHNode> root_;
    std::unique_ptr<BVHNode> buildNode(std::vector<Triangle> &triangles, int depth);
    bool intersectNode(const BVHNode *node, const Ray &ray, HitRecord &hit) const;
};

#endif // BVH_HPP