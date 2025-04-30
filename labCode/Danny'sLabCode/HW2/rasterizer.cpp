
#include "rasterizer.hpp"

#include <math.h>

#include <algorithm>
#include <array>
#include <opencv2/opencv.hpp>
#include <vector>

#include "Shader.hpp"
#include "Triangle.hpp"
#include <algorithm>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions) {
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices) {
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols) {
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f> &normals) {
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}

void rst::rasterizer::post_process_buffer() {
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            int index = get_index(x, y);
            for (int i = 0; i < 4; i++) {
                frame_buf[index] += ssaa_frame_buf[4 * index + i];
            }
            frame_buf[index] /= 4;
        }
    }
}

// Bresenham's line drawing algorithm
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end) {
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

    dx = x2 - x1;
    dy = y2 - y1;
    dx1 = fabs(dx);
    dy1 = fabs(dy);
    px = 2 * dy1 - dx1;
    py = 2 * dx1 - dy1;

    if (dy1 <= dx1) {
        if (dx >= 0) {
            x = x1;
            y = y1;
            xe = x2;
        } else {
            x = x2;
            y = y2;
            xe = x1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point, line_color);
        for (i = 0; x < xe; i++) {
            x = x + 1;
            if (px < 0) {
                px = px + 2 * dy1;
            } else {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
                    y = y + 1;
                } else {
                    y = y - 1;
                }
                px = px + 2 * (dy1 - dx1);
            }
            //            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point, line_color);
        }
    } else {
        if (dy >= 0) {
            x = x1;
            y = y1;
            ye = y2;
        } else {
            x = x2;
            y = y2;
            ye = y1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point, line_color);
        for (i = 0; y < ye; i++) {
            y = y + 1;
            if (py <= 0) {
                py = py + 2 * dx1;
            } else {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
                    x = x + 1;
                } else {
                    x = x - 1;
                }
                py = py + 2 * (dx1 - dy1);
            }
            //            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point, line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f) {
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

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

bool backFaceCulling(Eigen::Vector3f A, Eigen::Vector3f B, Eigen::Vector3f C){
    using namespace Eigen;
    Vector3f BminusA = (B - A);
    Vector3f CminusA = (C - A);
    Vector3f crossResult = BminusA.cross(CminusA);
    Vector3f lookVector = {0, 0, -1};
    if(lookVector.dot(crossResult) >= 0){
        return true;
    }
    return false;
}

// TODO: Task1 Implement this function
void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type, bool culling, bool anti_aliasing) {
    auto &buf = pos_buf[pos_buffer.pos_id];
    auto &ind = ind_buf[ind_buffer.ind_id];
    auto &col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto &i : ind) {
        Triangle t;

        std::array<Eigen::Vector4f, 3> mm{view * model * to_vec4(buf[i[0]], 1.0f),
                                          view * model * to_vec4(buf[i[1]], 1.0f),
                                          view * model * to_vec4(buf[i[2]], 1.0f)};

        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto &v) { return v.template head<3>(); });
        
        // TODO: Task1 Enable back face culling
        if (culling) {
            if(backFaceCulling(viewspace_pos[0], viewspace_pos[1], viewspace_pos[2])){
                continue;
            }
        }

        Eigen::Vector4f v[] = {mvp * to_vec4(buf[i[0]], 1.0f), mvp * to_vec4(buf[i[1]], 1.0f),
                               mvp * to_vec4(buf[i[2]], 1.0f)};

        // Homogeneous division
        for (auto &vec : v) {
            vec /= vec.w();
        }
        // Viewport transformation
        for (auto &vert : v) {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i) {
            t.setVertex(i, v[i]);
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);
/*************** update below *****************/
        rasterize_triangle(t, anti_aliasing);
    }
    if (anti_aliasing){
        post_process_buffer();
    }
}

Eigen::Matrix4f look_at(Eigen::Vector3f eye_pos){
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Vector3f z = (eye_pos - Eigen::Vector3f(0, 0, 0)).normalized();
    Eigen::Vector3f x = Eigen::Vector3f(0, 1, 0).cross(z).normalized();
    Eigen::Vector3f y = z.cross(x).normalized();

    Eigen::Matrix4f rotate;
    rotate << x.x(), x.y(), x.z(), 0, y.x(), y.y(), y.z(), 0, z.x(), z.y(), z.z(), 0, 0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1, -eye_pos[2], 0, 0, 0, 1;

    view = rotate * translate * view;
    return view;
}

void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList, bool culling, rst::Shading shading, bool shadow) {
    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;

    std::vector<Eigen::Matrix4f> lightspaceTransformations;
    std::vector<light> viewspace_lights;
    for (const auto &l : lights) {
        light view_space_light;
        Eigen::Matrix4f lightCamera = look_at(l.position);
        lightCamera = projection * lightCamera * model;
        lightspaceTransformations.push_back(lightCamera);
        view_space_light.position = (view * to_vec4(l.position, 1.0f)).head<3>();
        view_space_light.intensity = l.intensity;
        viewspace_lights.push_back(view_space_light);
    }
    std::vector<std::vector<float>> shadowBuffers;
    if(shadow){
        for(Eigen::Matrix4f trans : lightspaceTransformations){
            std::vector<float> shadowBuf = std::vector<float>();
            shadowBuf.resize(shadow_buf.size());
            std::fill(shadowBuf.begin(), shadowBuf.end(), std::numeric_limits<float>::infinity());
            for(const auto &t : TriangleList){
                Triangle newtri = *t;
                std::array<Eigen::Vector4f, 3> mm{(view * model * t->v[0]), (view * model * t->v[1]), (view * model * t->v[2])};
                std::array<Eigen::Vector3f, 3> lightPos;

                std::transform(mm.begin(), mm.end(), lightPos.begin(), [](auto &v) { return v.template head<3>(); });
                if (culling) {
                    if(backFaceCulling(lightPos[0], lightPos[1], lightPos[2])){
                        continue;
                    }
                }
                Eigen::Vector4f v1[] = {trans * t->v[0], trans * t->v[1], trans * t->v[2]};
                // Homogeneous division
                for (auto &vec : v1) {
                    vec.x() /= vec.w();
                    vec.y() /= vec.w();
                    vec.z() /= vec.w();
                }
                Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
                Eigen::Vector4f n[] = {inv_trans * to_vec4(t->normal[0], 0.0f), inv_trans * to_vec4(t->normal[1], 0.0f),
                                    inv_trans * to_vec4(t->normal[2], 0.0f)};

                for (auto &vert : v1) {
                    vert.x() = 0.5 * width * (vert.x() + 1.0);
                    vert.y() = 0.5 * height * (vert.y() + 1.0);
                    vert.z() = vert.z() * f1 + f2;
                }
                for (int i = 0; i < 3; ++i) {
                    newtri.setVertex(i, v1[i]);
                }
                for (int i = 0; i < 3; ++i) {
                    newtri.setNormal(i, n[i].head<3>());
                }
                std::array<Vector4f, 3UL> v = newtri.toVector4();
                Vector4f a = newtri.a();
                Vector4f b = newtri.b();
                Vector4f c = newtri.c();

                float minX = std::min({a.x(), b.x(), c.x()});
                float maxX = std::max({a.x(), b.x(), c.x()});
                float minY = std::min({a.y(), b.y(), c.y()});
                float maxY = std::max({a.y(), b.y(), c.y()});

                auto inTri = [&](float x, float y){
                    auto area = [](int x1, int y1, int x2, int y2, int x3, int y3){
                        return abs((x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))/2.0);
                    };
                    //only works with whole numbers
                    //so mutiply by 4 to convert 0.25, 0.5 and 0.75 to whole numbers
                    // 0.25 * 4 = 1, 0.5 * 4 = 2, 0.75 * 4 = 3
                    x *= 4;
                    y *= 4;
                    float aX = a.x() * 4;
                    float aY = a.y() * 4;
                    float bX = b.x() * 4;
                    float bY = b.y() * 4;
                    float cX = c.x() * 4;
                    float cY = c.y() * 4;


                    float areaABC = area(aX, aY, bX, bY, cX, cY);

                    float areaPBC = area(x, y, bX, bY, cX, cY);
                    float areaAPC = area(aX, aY, x, y, cX, cY);
                    float areaABP = area(aX, aY, bX, bY, x, y);
                    return (areaABC == areaPBC + areaAPC + areaABP);
                };

                float w_reciprocal, z_interpolated;
                for(float x = minX; x < maxX + 1; ++x){
                    for(float y = minY; y < maxY + 1; ++y){
                        if(inTri(x + 0.5, y + 0.5)){
                            auto[alpha, beta, gamma] = computeBarycentric2D(x+0.5, y+0.5, newtri.v);
                            w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;
                            if(z_interpolated < shadowBuf[get_index(x, y)]){
                                shadowBuf[get_index(x, y)] = z_interpolated;
                            }
                        }
                    }
                }
            }
            shadowBuffers.push_back(shadowBuf);
        }
    }
    for (const auto &t : TriangleList) {
        Triangle newtri = *t;

        std::array<Eigen::Vector4f, 3> mm{(view * model * t->v[0]), (view * model * t->v[1]), (view * model * t->v[2])};

        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto &v) { return v.template head<3>(); });
        // TODO: Task1 Enable back face culling
        if (culling) {
            if(backFaceCulling(viewspace_pos[0], viewspace_pos[1], viewspace_pos[2])){
                continue;
            }
        }
        Eigen::Vector4f v[] = {mvp * t->v[0], mvp * t->v[1], mvp * t->v[2]};
        // Homogeneous division
        for (auto &vec : v) {
            vec.x() /= vec.w();
            vec.y() /= vec.w();
            vec.z() /= vec.w();
        }
        Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
        Eigen::Vector4f n[] = {inv_trans * to_vec4(t->normal[0], 0.0f), inv_trans * to_vec4(t->normal[1], 0.0f),
                               inv_trans * to_vec4(t->normal[2], 0.0f)};

        // Viewport transformation
        for (auto &vert : v) {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }
        for (int i = 0; i < 3; ++i) {
            // screen space coordinates
            newtri.setVertex(i, v[i]);
        }
        for (int i = 0; i < 3; ++i) {
            // view space normal
            newtri.setNormal(i, n[i].head<3>());
        }
        newtri.setColor(0, 148, 121.0, 92.0);
        newtri.setColor(1, 148, 121.0, 92.0);
        newtri.setColor(2, 148, 121.0, 92.0);
        // Also pass view space vertice position
        rasterize_triangle(newtri, viewspace_pos, viewspace_lights, shadowBuffers, mvp, lightspaceTransformations, shading, shadow);
    }
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f &vert1,
                                   const Eigen::Vector3f &vert2, const Eigen::Vector3f &vert3, float weight) {
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f &vert1,
                                   const Eigen::Vector2f &vert2, const Eigen::Vector2f &vert3, float weight) {
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

// TODO: Task1 Implement this function
void rst::rasterizer::rasterize_triangle(const Triangle &t, bool anti_aliasing) {
    using namespace std;
    using namespace Eigen;

    array<Vector4f, 3UL> v = t.toVector4();
    Vector4f a = t.a();
    Vector4f b = t.b();
    Vector4f c = t.c();

    // 1. Find out the bounding box of current triangle.
    float minX = min({a.x(), b.x(), c.x()});
    float maxX = max({a.x(), b.x(), c.x()});

    float minY = min({a.y(), b.y(), c.y()});
    float maxY = max({a.y(), b.y(), c.y()});

    auto inTri = [&](float x, float y){
        auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
        if(alpha < 0 || beta < 0 || gamma < 0){
            return false;
        }
        return true;
    };

    float w_reciprocal, z_interpolated;
    Vector3f colourA = t.color[0];
    Vector3f colourB = t.color[1];
    Vector3f colourC = t.color[2];
    Vector3f localColour;
    Vector2i point;
    // 2. Iterate through the pixel and find if the current pixel is inside the triangle
    // Subpixel sampling if do anti-aliasing
    for(float x = minX; x < maxX + 1; ++x){
        for(float y = minY; y < maxY + 1; ++y){
            if(anti_aliasing){
                int numInTriangle = 0;
                float cummalativeZ_interpolated = 0;
                localColour = {0, 0, 0};
                for(float i = 0.25; i < 1; i += 0.5){
                    for(float j = 0.25; j < 1; j += 0.5){
                        if(inTri(x + i, y + j)){
                            numInTriangle += 1;
                            auto[alpha, beta, gamma] = computeBarycentric2D(x+i, y+j, t.v);
                            w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;
                            cummalativeZ_interpolated += z_interpolated;
                            localColour += ((alpha * colourA) + (beta * colourB) + (gamma * colourC));
                        }
                    }
                }
                localColour /= 4;
                cummalativeZ_interpolated /= numInTriangle;
                if(numInTriangle > 0 && cummalativeZ_interpolated < depth_buf[get_index(x, y)]){
                    depth_buf[get_index(x, y)] = z_interpolated;
                    localColour = localColour * 255;
                    point = {x, y};
                    set_pixel(point, localColour);
                }

            }
            else{
                if(inTri(x + 0.5, y + 0.5)){
                    // 3. If so, use the following code to get the interpolated z value.
                    // auto[alpha, beta, gamma] = computeBarycentric2D(x+0.5, y+0.5, t.v);
                    // float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    // float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    // z_interpolated *= w_reciprocal;
                    auto[alpha, beta, gamma] = computeBarycentric2D(x+0.5, y+0.5, t.v);
                    w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    // 4. Check if the interpolated depth is closer than the value in the depth buffer.
                    if(z_interpolated < depth_buf[get_index(x, y)]){
                        // 5. If so, update the depth buffer and set the current pixel to the color of the interpolated color of vertices
                        // set_pixel(Vector2i(x, y), interpolated_color);
                        depth_buf[get_index(x, y)] = z_interpolated;
                        localColour = ((alpha * colourA) + (beta * colourB) + (gamma * colourC));
                        localColour = localColour * 255;
                        point = {x, y};
                        set_pixel(point, localColour);
                    }
                }
            }
        }
    }
}

// TODO: Task2 Implement this function
void rst::rasterizer::rasterize_triangle(const Triangle &t, const std::array<Eigen::Vector3f, 3> &view_pos,
                                         const std::vector<light> &view_lights, std::vector<std::vector<float>> shadowBuffers,
                                         Eigen::Matrix4f MVP, std::vector<Eigen::Matrix4f> lightTransforms,
                                         rst::Shading shading, bool shadow) {
    using namespace std;
    using namespace Eigen;

    array<Vector4f, 3UL> v = t.toVector4();
    Vector4f a = t.a();
    Vector4f b = t.b();
    Vector4f c = t.c();
    // Find the bounding box of the triangle.
    float minX = min({a.x(), b.x(), c.x()});
    float maxX = max({a.x(), b.x(), c.x()});

    float minY = min({a.y(), b.y(), c.y()});
    float maxY = max({a.y(), b.y(), c.y()});
    
    auto inTri = [&](float x, float y){
        auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
        if(alpha < 0 || beta < 0 || gamma < 0){
            return false;
        }
        return true;
    };

    float w_reciprocal, z_interpolated, weight;
    Vector3f colourA = t.color[0];
    Vector3f colourB = t.color[1];
    Vector3f colourC = t.color[2];
    Vector3f pixel_color, pixelColourA, pixelColourB, pixelColourC;
    Vector2i point;
    fragment_shader_payload payload, payloadA, payloadB, payloadC;
    if (shading == rst::Shading::Flat) {
        std::vector<light> localLights = view_lights;
        if (shadow) {
            Eigen::Vector4f pos = {a.x(), a.y(), a.z(), 1};
            for(int i = 0; i < view_lights.size(); ++i){
                Eigen::Vector4f lightSpacePos = lightTransforms[i] * (MVP.inverse() * pos);
                //use > instead of != to try to reduce floating point inaccuracies
                if(shadowBuffers[i][get_index(lightSpacePos.x(), lightSpacePos.y())] > lightSpacePos.z()){
                    localLights[i].intensity = {0, 0, 0};
                }
            }
        }
        //will use colour, texture and shading at vertex A
        payload = fragment_shader_payload(colourA, t.normal[0].normalized(), t.tex_coords[0], localLights, texture ? &*texture : nullptr);
        payload.view_pos = {a.x(), a.y(), a.z()};
        pixel_color = fragment_shader(payload);
    } else if (shading == rst::Shading::Gouraud) {
        std::vector<light> localLightsA = view_lights;
        std::vector<light> localLightsB = view_lights;
        std::vector<light> localLightsC = view_lights;
        if (shadow) {
            Eigen::Vector4f posA = {a.x(), a.y(), a.z(), 1};
            Eigen::Vector4f posB = {b.x(), b.y(), b.z(), 1};
            Eigen::Vector4f posC = {c.x(), c.y(), c.z(), 1};
            for(int i = 0; i < view_lights.size(); ++i){
                Eigen::Vector4f lightSpacePosA = lightTransforms[i] * (MVP.inverse() * posA);
                Eigen::Vector4f lightSpacePosB = lightTransforms[i] * (MVP.inverse() * posB);
                Eigen::Vector4f lightSpacePosC = lightTransforms[i] * (MVP.inverse() * posC);
                //use > instead of != to try to reduce floating point inaccuracies
                if(shadowBuffers[i][get_index(lightSpacePosA.x(), lightSpacePosA.y())] > lightSpacePosA.z()){
                    localLightsA[i].intensity = {0, 0, 0};
                }
                if(shadowBuffers[i][get_index(lightSpacePosB.x(), lightSpacePosB.y())] > lightSpacePosB.z()){
                    localLightsB[i].intensity = {0, 0, 0};
                }
                if(shadowBuffers[i][get_index(lightSpacePosC.x(), lightSpacePosC.y())] > lightSpacePosC.z()){
                    localLightsC[i].intensity = {0, 0, 0};
                }
            }
        }
        //get shader for vertex A
        fragment_shader_payload payloadA(colourA, t.normal[0].normalized(), t.tex_coords[0], localLightsA, texture ? &*texture : nullptr);
        payloadA.view_pos = {a.x(), a.y(), a.z()};
        pixelColourA = fragment_shader(payloadA);
        //get shader for vertex B
        fragment_shader_payload payloadB(colourB, t.normal[1].normalized(), t.tex_coords[1], localLightsB, texture ? &*texture : nullptr);
        payloadB.view_pos = {b.x(), b.y(), b.z()};
        pixelColourA = fragment_shader(payloadB);
        //get shader for vertex C
        fragment_shader_payload payloadC(colourC, t.normal[2].normalized(), t.tex_coords[2], localLightsC, texture ? &*texture : nullptr);
        payloadC.view_pos = {c.x(), c.y(), c.z()};
        pixelColourA = fragment_shader(payloadC);
    } 

    // iterate through the pixel and find if the current pixel is inside the triangle
    for(float x = minX; x < maxX + 1; ++x){
        for(float y = minY; y < maxY + 1; ++y){
            if(inTri(x + 0.5, y + 0.5)){
                // If so, use the following code to get the interpolated z value.
                // auto[alpha, beta, gamma] = computeBarycentric2D(x+0.5, y+0.5, t.v);
                // float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma /v[2].w());
                // float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() /
                // v[2].w(); z_interpolated *= w_reciprocal;
                auto[alpha, beta, gamma] = computeBarycentric2D(x+0.5, y+0.5, t.v);
                w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                // Check if the interpolated depth is closer than the value in the depth buffer.
                if(z_interpolated < depth_buf[get_index(x, y)]){
                    // If so, update the depth buffer
                    depth_buf[get_index(x, y)] = z_interpolated;
                    if (shading == rst::Shading::Gouraud) {
                        pixel_color = (alpha * pixelColourA) + (beta * pixelColourB) + (gamma * pixelColourC);
                    } else if (shading == rst::Shading::Phong) {
                        // and calculate interpolated_color, interpolated_normal, interpolated_texcoords and interpolated_shadingcoords
                        // float weight = alpha + beta + gamma;
                        Vector3f interpolated_color = ((alpha * colourA) + (beta * colourB) + (gamma * colourC));
                        Vector2f interpolated_texcoords = (alpha * t.tex_coords[0]) + (beta * t.tex_coords[1]) + (gamma * t.tex_coords[2]);
                        Vector3f interpolated_normal = (alpha * t.normal[0]) + (beta * t.normal[1]) + (gamma * t.normal[2]);
                        Vector3f interpolated_shadingcoords = {x, y, z_interpolated};
                        // pass them to the fragment_shader_payload
                        // fragment_shader_payload payload(interpolated_color, interpolated_normal.normalized(), interpolated_texcoords,
                        // view_lights, texture ? &*texture : nullptr); payload.view_pos = interpolated_shadingcoords;
                        std::vector<light> localLights = view_lights;
                        // modify the color if do shadow mapping
                        if (shadow) {
                            Eigen::Vector4f pos = {x, y, z_interpolated, 1};
                            for(int i = 0; i < view_lights.size(); ++i){
                                Eigen::Vector4f lightSpacePos = lightTransforms[i] * (MVP.inverse() * pos);
                                //use > instead of != to try to reduce floating point inaccuracies
                                if(shadowBuffers[i][get_index(lightSpacePos.x(), lightSpacePos.y())] > lightSpacePos.z()){
                                    localLights[i].intensity = {0, 0, 0};
                                }
                            }
                        }
                        fragment_shader_payload payload(interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, localLights, texture ? &*texture : nullptr);
                        payload.view_pos = interpolated_shadingcoords;
                        // Call the fragment shader to get the pixel color
                        pixel_color = fragment_shader(payload);
                        // set the pixel color to the frame buffer.
                    }
                    point = {x, y};
                    set_pixel(point, pixel_color);
                }
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f &m) {
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f &v) {
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f &p) {
    projection = p;
}

void rst::rasterizer::set_lights(const std::vector<light> &l) {
    lights = l;
}

void rst::rasterizer::set_shadow_view(const Eigen::Matrix4f &v) {
    shadow_view = v;
}

void rst::rasterizer::set_shadow_buffer(const std::vector<float> &shadow_buffer) {
    std::copy(shadow_buffer.begin(), shadow_buffer.end(), this->shadow_buf.begin());
}

void rst::rasterizer::clear(rst::Buffers buff) {
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color) {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(ssaa_frame_buf.begin(), ssaa_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(ssaa_depth_buf.begin(), ssaa_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h) {
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    shadow_buf.resize(w * h);
    ssaa_frame_buf.resize(4 * w * h);
    ssaa_depth_buf.resize(4 * w * h);
    texture = std::nullopt;
}

int rst::rasterizer::get_index(int x, int y) {
    return (height - y - 1) * width + x;
}

void rst::rasterizer::set_pixel(const Vector2i &point, const Eigen::Vector3f &color) {
    // old index: auto ind = point.y() + point.x() * width;
    int ind = (height - point.y() - 1) * width + point.x();
    frame_buf[ind] = color;
}

void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader) {
    vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader) {
    fragment_shader = frag_shader;
}
