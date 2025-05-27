#include "bvh.hpp"
#include <limits>
#include <algorithm>
#include <cmath>

// Moller-Trumbore intersection algorithm for triangles
bool Triangle::intersect(const Ray &ray, float &tOut, float &u, float &v) const {
    Eigen::Vector3f edge1 = v1 - v0;
    Eigen::Vector3f edge2 = v2 - v0;

    // Check if ray is parallel to the triangle
    float det = edge1.dot(ray.direction.cross(edge2));
    if (std::abs(det) < EPSILON)
        return false; // Ray is parallel so no hit

    // Calculate the barycentric coordinate u
    float invDet = 1.0f / det;
    u = invDet * (ray.origin - v0).dot(ray.direction.cross(edge2));
    if (u < -EPSILON || u > 1 + EPSILON)
        return false; // Hit outside triangle

    v = invDet * ray.direction.dot((ray.origin - v0).cross(edge1));
    if (v < -EPSILON || u + v > 1 + EPSILON)
        return false; // Hit outside triangle

    // Hit must be inside triangle, calculate t
    tOut = invDet * edge2.dot((ray.origin - v0).cross(edge1));
    return tOut > EPSILON; // Return true if t is positive, indicating a hit in front of the ray origin
}

Eigen::Vector3f Triangle::sample(float &pdf) const {
    float r1 = randomFloat();
    float r2 = randomFloat();
    float a = 1 - r1;
    float b = r1 * (1 - r2);
    float c = r1 * r2;
    pdf = 1.0f / area;
    return a * v0 + b * v1 + c * v2;
}

static void computeBounds(const std::vector<Triangle> &tris, Eigen::Vector3f &bmin, Eigen::Vector3f &bmax) {
    bmin = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
    bmax = Eigen::Vector3f::Constant(-std::numeric_limits<float>::infinity());
    for (const auto &tri : tris) {
        bmin = bmin.cwiseMin(tri.v0).cwiseMin(tri.v1).cwiseMin(tri.v2);
        bmax = bmax.cwiseMax(tri.v0).cwiseMax(tri.v1).cwiseMax(tri.v2);
    }
}

void BVHAccel::build(std::vector<Triangle> &triangles) {
    root_ = buildNode(triangles, 0);
}

std::unique_ptr<BVHNode> BVHAccel::buildNode(std::vector<Triangle> &triangles, int depth) {
    auto node = std::make_unique<BVHNode>();
    computeBounds(triangles, node->boundsMin, node->boundsMax);
    if (triangles.size() <= 1000 || depth > 20) {
        node->triangles = std::move(triangles);
        return node; // Leaf node
    }
    Eigen::Vector3f extent = node->boundsMax - node->boundsMin;
    int axis = (extent.x() > extent.y() && extent.x() > extent.z()) ? 0 :
               (extent.y() > extent.z()) ? 1 : 2;

    std::sort(triangles.begin(), triangles.end(), [axis](auto &a, auto &b) {
        float ca = (a.v0[axis] + a.v1[axis] + a.v2[axis]) / 3.0f;
        float cb = (b.v0[axis] + b.v1[axis] + b.v2[axis]) / 3.0f;
        return ca < cb;
    }
    );

    size_t mid = triangles.size() / 2;
    std::vector<Triangle> leftTris(triangles.begin(), triangles.begin() + mid);
    std::vector<Triangle> rightTris(triangles.begin() + mid, triangles.end());
    
    if (leftTris.empty() || rightTris.empty()) {
        node->triangles = std::move(triangles); // No split possible, make leaf
        return node;
    }
    node->left = buildNode(leftTris, depth + 1);
    node->right = buildNode(rightTris, depth + 1);
    return node; // Internal node
}

bool BVHAccel::intersect(const Ray &ray, HitRecord &hit) const {
    return intersectNode(root_.get(), ray, hit);
}

bool BVHAccel::intersectNode(const BVHNode *node, const Ray &ray, HitRecord &hit) const {
    Eigen::Vector3f invDir = ray.direction.cwiseInverse();
    Eigen::Vector3f t0 = (node->boundsMin - ray.origin).cwiseProduct(invDir);
    Eigen::Vector3f t1 = (node->boundsMax - ray.origin).cwiseProduct(invDir);
    Eigen::Vector3f tmin = t0.cwiseMin(t1);
    Eigen::Vector3f tmax = t0.cwiseMax(t1);
    if (tmin.maxCoeff() > tmax.minCoeff())
        return false; // No intersection with bounding box

    bool hitFound = false;
    if (node->isLeaf()) {
        for (const auto &tri : node->triangles) {
            float t, u, v;
            if (tri.intersect(ray, t, u, v) && t < hit.t) {
                hit.t = t;
                hit.position = ray.at(t);
                hit.normal = ((1 - u -v)*tri.n0 + u*tri.n1 + v*tri.n2).normalized();
                hit.textureCoords = (1 - u - v) * tri.t0 + u * tri.t1 + v * tri.t2;
                hit.mat = tri.mat;
                hit.material = tri.material;
                hitFound = true; // Found a hit
            } 
        }
    } else {
        if (intersectNode(node->left.get(), ray, hit)) hitFound = true;
        if (intersectNode(node->right.get(), ray, hit)) hitFound = true;
    }
    return hitFound;
}