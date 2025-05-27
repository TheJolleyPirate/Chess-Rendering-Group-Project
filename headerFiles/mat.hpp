#ifndef MAT_HPP
#define MAT_HPP

#include <Eigen/Dense>
#include <global.hpp>
#include <algorithm>
#include "bvh.hpp"
#include <random>

class Mat {
public:
    virtual ~Mat() = default;

    Eigen::Vector3f toWorld(const Eigen::Vector3f &local, const Eigen::Vector3f &N) const {
        Eigen::Vector3f B, C;
        if (std::fabs(N.x()) > std::fabs(N.y())){
            float invLen = 1.0f / std::sqrt(N.x() * N.x() + N.z() * N.z());
            C = Eigen::Vector3f(N.z() * invLen, 0.0f, -N.x() *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y() * N.y() + N.z() * N.z());
            C = Eigen::Vector3f(0.0f, N.z() * invLen, -N.y() *invLen);
        }
        B = C.cross(N);
        return local.x() * B + local.y() * C + local.z() * N;
    }

    virtual Eigen::Vector3f sample(
        const HitRecord &hit, 
        const Eigen::Vector3f &wo, 
        Eigen::Vector3f &wi,
        float &pdf
    ) const = 0;

    virtual Eigen::Vector3f evaluate(
        const HitRecord &hit, 
        const Eigen::Vector3f &wo, 
        const Eigen::Vector3f &wi
    ) const = 0;

    virtual float pdf(
        const HitRecord &hit, 
        const Eigen::Vector3f &wo, 
        const Eigen::Vector3f &wi
    ) const = 0;
};

class Lambertian : public Mat {
public:
    explicit Lambertian(const Material &material) : material_(material) {}

    Eigen::Vector3f sample(
        const HitRecord &hit, 
        const Eigen::Vector3f &wo, 
        Eigen::Vector3f &wi,
        float &pdf
    ) const override {
        float r1 = randomFloat();
        float r2 = randomFloat();
        float phi = 2 * M_PI * r1;

        float x = std::cos(phi) * std::sqrt(r2);
        float y = std::sin(phi) * std::sqrt(r2);
        float z = std::sqrt(1 - r2);
        Eigen::Vector3f local(x, y, z);

        wi = toWorld(local, hit.normal);
        pdf = (hit.normal.dot(wi) > 0.0f) ? (hit.normal.dot(wi) / M_PI) : 0.0f;

        Eigen::Vector3f albedo = material_.diffuseTexture.getColor(hit.textureCoords.x(), hit.textureCoords.y());
        return albedo / M_PI;
    }

    Eigen::Vector3f evaluate(
        const HitRecord &hit, 
        const Eigen::Vector3f &wo, 
        const Eigen::Vector3f &wi
    ) const override {
        Eigen::Vector3f albedo = material_.diffuseTexture.getColor(hit.textureCoords.x(), hit.textureCoords.y());
        return albedo / M_PI;
    }

    float pdf(
        const HitRecord &hit, 
        const Eigen::Vector3f &wo, 
        const Eigen::Vector3f &wi
    ) const override {
        return (hit.normal.dot(wi) > 0.0f) ? (hit.normal.dot(wi) / M_PI) : 0.0f;
    }

private:
    // Eigen::Vector3f albedo_;
    Material material_;
};

#endif // MAT_HPP