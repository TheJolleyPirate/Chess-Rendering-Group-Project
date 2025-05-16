#ifndef RAYTRACING_CYLINDER_H
#define RAYTRACING_CYLINDER_H

#include "Object.hpp"
#include "Vector.hpp"
#include "Bounds3.hpp"
#include "Material.hpp"


class Cylinder : public Object {
    // TODO: Define a cylinder
public:
    Vector3f baseCenter;
    Vector3f axis; // normalized direction of cylinder
    float radius, radius2;
    float height, area;
    Material *m;

    Cylinder(const Vector3f &bc, const Vector3f &a, const float &r, const float &h, Material* mt = new Material())
        : baseCenter(bc), axis(normalize(a)), radius(r), radius2(r * r), height(h), m(mt), area(2 * M_PI * r * h + 2 * M_PI * r * r) {}

    Intersection getIntersection(Ray ray) {
        Intersection result;
        result.happened = false;

        // Project ray onto the plane perpendicular to the cylinder axis to check side surface intersection
        Vector3f d = ray.direction - dotProduct(ray.direction, axis) * axis;
        Vector3f oc = ray.origin - baseCenter;
        Vector3f oc_proj = oc - dotProduct(oc, axis) * axis;

        // Solve the quadratic equation for intersection with the cylinder side
        float a = dotProduct(d, d);
        float b = 2 * dotProduct(d, oc_proj);
        float c = dotProduct(oc_proj, oc_proj) - radius2;

        float t0, t1;
        if (solveQuadratic(a, b, c, t0, t1)) {
            if (t0 < 0) t0 = t1;
            if (t0 >= 0) {
                // Check if the intersection point is within the height of the cylinder
                Vector3f point_hit = ray.origin + ray.direction * t0;
                float h = dotProduct(point_hit - baseCenter, axis);

                if (h >= 0 && h <= height) {
                    // Set intersection details
                    result.happened = true;
                    result.coords = point_hit;
                    result.normal = normalize((point_hit - baseCenter) - axis * h);
                    result.tnear = t0;
                    result.material = this->m;
                    result.obj = this;
                    return result;
                    }
            }
        }

        // Check for intersection with the cylinders bottom cap
        float t_bottom = dotProduct((baseCenter - ray.origin), axis) / dotProduct(ray.direction, axis);
        if (t_bottom >= 0) {
            Vector3f point_hit = ray.origin + ray.direction * t_bottom;
            if (dotProduct(point_hit - baseCenter, point_hit - baseCenter) <= radius2) {
                result.happened = true;
                result.coords = point_hit;
                result.normal = -axis;
                result.tnear = t_bottom;
                result.material = this->m;
                result.obj = this;
                return result;
            }
        }

        // Check for intersection with the cylinders top cap
        Vector3f topCenter = baseCenter + axis * height;
        float t_top = dotProduct((topCenter - ray.origin), axis) / dotProduct(ray.direction, axis);
        if (t_top >= 0) {
            Vector3f point_hit = ray.origin + ray.direction * t_top;
            if (dotProduct(point_hit - topCenter, point_hit - topCenter) <= radius2) {
                result.happened = true;
                result.coords = point_hit;
                result.normal = axis;
                result.tnear = t_top;
                result.material = this->m;
                result.obj = this;
                return result;
            }
        }
        return result; // No intersection found
    }

    void getSurfaceProperties(const Vector3f &P, const Vector3f &I, const uint32_t &index, const Vector2f &uv, Vector3f &N, Vector2f &st) const {
        // Calculate the normal at the intersection point
        Vector3f rel_pos = P - baseCenter;
        float h = dotProduct(rel_pos, axis);
        Vector3f circle_pos = baseCenter + axis * h;
        N = normalize((P - circle_pos) - axis * h);
    }

    Vector3f evalDiffuseColor(const Vector2f &st) const {
        return m->getColor();
    }

    Bounds3 getBounds() {
        Vector3f min = baseCenter - Vector3f(radius, 0, radius);
        Vector3f max = baseCenter + Vector3f(radius, height, radius);
        return Bounds3(min, max);
    }

    void Sample(Intersection &pos, float &pdf) {
        // Sample a point on the cylinder surface
        float theta = 2.0 * M_PI * get_random_float();
        float h = height * get_random_float();
        Vector3f point = baseCenter + axis * h + radius * Vector3f(cos(theta), 0, sin(theta));
        pos.coords = point;
        pos.normal = normalize(Vector3f(cos(theta), 0, sin(theta)));
        pos.obj = this;
        pos.material = m;
        pdf = 1.0f / area;
    }

    float getArea() {
        return area;
    }

    bool hasEmit() {
        return m->hasEmission();
    }
};
    
#endif // RAYTRACING_CYLINDER_H