#ifndef RAY_HPP
#define RAY_HPP

#include <Eigen/Dense>

/*by Matthew Reynolds u6949604*/
struct Ray {
    Eigen::Vector3f origin;
    Eigen::Vector3f direction;

    Ray() = default;
    Ray(const Eigen::Vector3f &o, const Eigen::Vector3f &d)
        : origin(o), direction(d.normalized()) {}

    Eigen::Vector3f at(float t) const {
        return origin + t * direction;
    }
};

#endif //RAY_HPP