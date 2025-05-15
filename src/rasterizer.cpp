#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Eigen/Dense"

#include <headerFiles/rasterizer.hpp>
#include <headerFiles/scene.hpp>
#include <headerFiles/light.hpp>
#include <headerFiles/object.hpp>
#include <headerFiles/camera.hpp>

using namespace std;
using namespace Eigen;

rst::Rasterizer::Rasterizer(int w, int h) : width(w), height(h) {
    frameBuffer.resize(w * h);
    depthBuffer.resize(w * h);
    ssaaFrameBuffer.resize(4 * w * h);
    ssaaDepthBuffer.resize(4 * w * h);
    texture = std::nullopt;
}

void rst::Rasterizer::setView(const Eigen::Matrix4f& v) {
    view = v;
}

void rst::Rasterizer::setProjection(const Eigen::Matrix4f& p) {
    projection = p;
}

void rst::Rasterizer::setFragmentShader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader) {
    fragment_shader = frag_shader;
}

// Clear the frame buffer and/or depth buffer
void rst::Rasterizer::clear(Buffers buff) {
    if ((buff & Buffers::Colour) == Buffers::Colour) {
        std::fill(frameBuffer.begin(), frameBuffer.end(), Eigen::Vector3f(0, 0, 0));
        std::fill(ssaaFrameBuffer.begin(), ssaaFrameBuffer.end(), Eigen::Vector3f(0, 0, 0));
    }
    if ((buff & Buffers::Depth) == Buffers::Depth) {
        std::fill(depthBuffer.begin(), depthBuffer.end(), std::numeric_limits<float>::infinity());
        std::fill(ssaaDepthBuffer.begin(), ssaaDepthBuffer.end(), std::numeric_limits<float>::infinity());
    }
}

// Convert a 3D vector to a 4D vector
auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f) {
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


// Rasterize the objects in the scene
void rst::Rasterizer::rasterizeObjects(Scene scene){
    for (auto& object : scene.objects) {
        if (object.material.diffuseTextureFile != "") {
            set_texture(Texture(object.material.diffuseTextureFile));
        }
        auto faces = object.faces;
        draw(faces, scene.lights);
    }
}

void rst::Rasterizer::draw(std::vector<std::shared_ptr<Face>> faces, std::vector<Light> lights) {
    // Needed to manually map z screen positions to [nearPlane, farPlane] for current test camera
    float f1 = (50 - 0.1) / 2.0f;
    float f2 = (50 + 0.1) / 2.0f;

    // Convert lights to view space
    std::vector<Light> viewspace_lights;
    for (auto &light : lights) {
        Eigen::Vector4f lightPos = view * to_vec4(light.position);
        viewspace_lights.push_back(Light(lightPos.head<3>(), light.intensity));
    }

    // Extract vertices from the face
    for (auto &face : faces) {
        auto vertices = face->getVertices();
        if (vertices.size() != 3) {
            std::cerr << "Error: Face is not a triangle!" << std::endl;
            return;
        }

        // Convert vertices to viewspace and screen space
        Eigen::Matrix4f model = Eigen::Matrix4f::Identity(); // Assuming face is already in world space
        Eigen::Matrix4f mvp = model * view * projection;

        std::vector<Eigen::Vector3f> viewspace_vertices;
        std::vector<Vertex> screenspace_vertices;
        for (int i = 0; i < 3; i++) {
            // Convert to view space
            Eigen::Vector4f view_pos = view * to_vec4(vertices[i]->position);
            viewspace_vertices.push_back(view_pos.head<3>());

            // Convert to screen space
            Eigen::Vector4f screen_pos = mvp * to_vec4(vertices[i]->position);
            screen_pos /= screen_pos.w(); // Perpective divide

            // Map NDC to screen space
            screen_pos.x() = 0.5 * width * (screen_pos.x() + 1.0);
            screen_pos.y() = 0.5 * height * (screen_pos.y() + 1.0);
            screen_pos.z() = (screen_pos.z() * f1) + f2;

            // Create a new vertex with the screen position and original attributes
            Vertex newVertex = *vertices[i];
            newVertex.position = screen_pos.head<3>();
            screenspace_vertices.push_back(newVertex);
        }
    }
}

// TODO: if we don't know what shape the face is
void rasterizeFace(Face face){
}

// Check if a point (x, y) is inside a triangle defined by three vertices
static bool insideTriangle(float x, float y, const Vector4f *_v) {
    Vector3f v[3];
    for (int i = 0; i < 3; i++)
        v[i] = {_v[i].x(), _v[i].y(), 1.0};
    Vector3f p(x, y, 1.);
    Vector3f f0, f1, f2;
    f0 = (p - v[0]).cross(v[1] - v[0]);
    f1 = (p - v[1]).cross(v[2] - v[1]);
    f2 = (p - v[2]).cross(v[0] - v[2]);
    if (f0.dot(f1) > 0 && f1.dot(f2) > 0)
        return true;
    return false;
}

// Compute barycentric coordinates for a point (x, y) inside a triangle defined by three vertices
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f *v) {
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) /
               (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() -
                v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) /
               (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() -
                v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) /
               (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() -
                v[1].x() * v[0].y());
    return {c1, c2, c3};
}

// Interpolates a 3D vector (e.g., color, normal) using barycentric coordinates
static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f &vert1,
    const Eigen::Vector3f &vert2, const Eigen::Vector3f &vert3, float weight) {
return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

// If we know the face is a triangle
void rst::Rasterizer::rasterizeTriangle(std::vector<Vertex> &vertices, std::vector<Eigen::Vector3f> &view_pos, std::vector<Light> &view_lights) {
    // Convert vertices to 4D homogeneous coordinates
    Eigen::Vector4f v[3];
    for (int i = 0; i < 3; i++) {
        v[i] = to_vec4(vertices[i].position);
    }

    // Compute bounding box
    float minX = std::max(0.0f, std::floor(std::min({v[0].x(), v[1].x(), v[2].x()})));
    float maxX = std::min((float)(width - 1), std::ceil(std::max({v[0].x(), v[1].x(), v[2].x()})));
    float minY = std::max(0.0f, std::floor(std::min({v[0].y(), v[1].y(), v[2].y()})));
    float maxY = std::min((float)(height - 1), std::ceil(std::max({v[0].y(), v[1].y(), v[2].y()})));

    // Subpixel sampling offsets for 2x2 SSAA
    Eigen::Vector2f ssaa_offset[4] = {
        Eigen::Vector2f(0.25, 0.25), // Top-left
        Eigen::Vector2f(0.75, 0.25), // Top-right
        Eigen::Vector2f(0.25, 0.75), // Bottom-left
        Eigen::Vector2f(0.75, 0.75)  // Bottom-right
    };

    // Compute to colours of the triangle's vertices
    Eigen::Vector3f vertex_colours[3];
    for (int j = 0; j < 3; j++) {
        fragment_shader_payload payload(vertices[j].colour,
                                        vertices[j].computeNormal(),
                                        vertices[j].textureCoordinates,
                                        view_lights,
                                        texture ? &*texture : nullptr);
        payload.view_pos = view_pos[j];
        Eigen::Vector3f vertex_colour = fragment_shader(payload);
        vertex_colours[j] = vertex_colour;
    }

    // Iterate over each pixel in the bounding box
    for (int x = minX; x <= maxX; x++) {
        for (int y = minY; y <= maxY; y++) {
            // Iterate over each subpixel
            for (int i = 0; i < 4; i++) {
                float px = x + ssaa_offset[i].x();
                float py = y + ssaa_offset[i].y();

                // Check if the subpixel is inside the triangle
                if (insideTriangle(px, py, v)) {
                    // Compute barycentric coordinates
                    auto [alpha, beta, gamma] = computeBarycentric2D(px, py, v);

                    // Interpolate depth
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + 
                                            beta * v[1].z() / v[1].w() + 
                                            gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    // Compute index for subpixel in the SSAA buffer
                    int index = 4 * getIndex(x, y) + i;

                    // Perform depth test
                    if (z_interpolated < ssaaDepthBuffer[index]) {
                        ssaaDepthBuffer[index] = z_interpolated;

                        // Compute subpixel colour -- default to Gouraud shading for now
                        Eigen::Vector3f pixel_colour = interpolate(alpha, beta, gamma,
                            vertex_colours[0], vertex_colours[1], vertex_colours[2], alpha + beta + gamma);

                        // Set pixel colour in the SSAA buffer
                        ssaaFrameBuffer[index] = pixel_colour;
                    }
                }
            }
        }
    }
    // Perform post-processing to average the SSAA samples
    postProcessBuffer();
}

// Post-process the SSAA buffer to average the samples and store them in the main frame buffer
void rst::Rasterizer::postProcessBuffer() {
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            int index = getIndex(x, y);
            for (int i = 0; i < 4; i++) {
                frameBuffer[index] += ssaaFrameBuffer[4 * index + i];
            }
            frameBuffer[index] /= 4;
        }
    }
}

// Convert a 2D screen coordinate (x, y) to a 1D index in the frame buffer
int rst::Rasterizer::getIndex(int x, int y) {
    return (height - y - 1) * width + x;
}

// Set a pixel in the frame buffer to a specific color
void rst::Rasterizer::setPixel(const Vector2i& position, const Eigen::Vector3f& colour) {
    int index = getIndex(position.x(), position.y());
    frameBuffer[index] = colour;
}