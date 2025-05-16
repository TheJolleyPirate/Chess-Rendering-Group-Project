//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

const float EPSILON = 1e-2f;

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}


void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            pos.happened=true;  // area light that has emission exists
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

// Compute the colour of surface at intersection using the Phong illumination model
Vector3f Scene::phongShader(const Intersection &inter, const Vector3f &v) const
{
    // Material properties
    float Ka = 0.001f;
    float Kd = inter.material->Kd;
    float Ks = inter.material->Ks;
    float specularExponent = inter.material->specularExponent;
    Vector3f colour = inter.obj->evalDiffuseColor(inter.tcoords);

    Vector3f result_colour = {0, 0, 0};

    // Inputs to Blinn shading model
    Vector3f p = inter.coords;
    Vector3f n = inter.normal;

    // Ambient light component
    Vector3f La = Ka * colour;

    for (auto &light : get_lights()) {
        // Remaining inputs to Blinn shading model
        Vector3f l = (light->position - p).normalized();
        Vector3f r = reflect(-l, n);

        // Check if object is in shadow
        float light_distance = (light->position - p).norm();
        Ray shadowRay(p + (l * EPSILON), l);
        auto shadowInter = intersect(shadowRay);
        if (shadowInter.happened && (shadowInter.coords - p).norm() < light_distance) 
            continue; // Shadow ray hit an object before reaching the light

        // Diffuse light component
        float diff = std::max(0.0f, dotProduct(n, l));
        Vector3f Ld = Kd * diff * colour * light->intensity;
        
        // Specular light component
        float spec = std::pow(std::max(0.0f, dotProduct(v, r)), specularExponent);
        Vector3f Ls = Ks * spec * Vector3f(2, 2, 2) * light->intensity;
            
        // Accumulate the light components
        result_colour += Ld + Ls;
    }
    // Add the ambient light component
    result_colour += La;

    return result_colour;
}

// Compute the colour of glass surface
Vector3f Scene::glassShader(const Intersection &inter, const Vector3f &I, int depth) const {
    Vector3f result_colour = {0, 0, 0};

    // Compute Fresnel reflection coefficient
    float kr = fresnel(I, inter.normal, inter.material->ior);

    // Compute reflection and refraction directions
    Vector3f reflect_dir = normalize(reflect(I, inter.normal));
    Vector3f refract_dir = normalize(refract(I, inter.normal, inter.material->ior));

    // Reflection component
    Ray reflectRay(inter.coords + reflect_dir * EPSILON, reflect_dir);
    Vector3f reflect_colour = castRay(reflectRay, depth + 1);

    // Refraction component
    Vector3f refract_colour = Vector3f(0);
    if (kr < 1.0f) { // Only compute refraction if not total internal reflection
        Ray refractRay(inter.coords + refract_dir * EPSILON, refract_dir);
        refract_colour = castRay(refractRay, depth + 1);
    } 

    // Combine reflection and refraction components
    result_colour = reflect_colour * kr + refract_colour * (1 - kr);
    return result_colour;
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Vector3f hitColor = Vector3f(0);

    auto inter = intersect(ray);
    if (!inter.happened)
        return backgroundColor;

    if (inter.material->m_type == EMIT) 
        return inter.material->m_emission;

    else if (inter.material->m_type == DIFFUSE || TASK_N<3) {
        // Task 5: Soft Shadows
        int light_samples = 16; // Number of samples for area light
        for (int i = 0; i < light_samples && TASK_N >= 5; ++i) {
            Intersection lightInter;
            float pdf_light = 0.0f;
            sampleLight(lightInter, pdf_light);  // sample a point on the area light
            
            // Compute light properties
            Vector3f lightDir = (lightInter.coords - inter.coords).normalized();
            float lightDistance = (lightInter.coords - inter.coords).norm();
            Vector3f emit = lightInter.material->m_emission;

            // Check if the objet is in shadow
            Ray shadowRay(inter.coords + lightDir * EPSILON, lightDir);
            Intersection shadowInter = intersect(shadowRay);
            if (shadowInter.happened && (shadowInter.coords - inter.coords).norm() < lightDistance)
                continue; // Shadow ray hit an object before reaching the light

            // Compute the surface colour from area light
            float dotLN = std::max(0.0f, dotProduct(lightDir, inter.normal));
            Vector3f diff = inter.obj->evalDiffuseColor(inter.tcoords);
            Vector3f m_eval = inter.material->eval(lightDir, inter.normal);
            float distance2 = lightDistance * lightDistance;
            
            hitColor += diff * emit * m_eval * dotLN / distance2 / pdf_light;
        }
        // Average the light samples
        hitColor = hitColor / light_samples;
        

        // Task 1.3: Basic Shading
        Vector3f viewDir = -ray.direction;
        hitColor += phongShader(inter, viewDir);
    } 
    
    else if (inter.material->m_type == GLASS && TASK_N>=3) {
        // Task 3: Glass Material
        if (depth > 5) return Vector3f(0); // Max recursion depth
        hitColor = glassShader(inter, ray.direction, depth);
    }

    return hitColor;
}
