#include "rasterizer.hpp"

#include <math.h>

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <type_traits>
#include <cstdlib>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
  auto id = get_next_id();
  pos_buf.emplace(id, positions);

  return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
  auto id = get_next_id();
  ind_buf.emplace(id, indices);

  return {id};
}

// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
// TODO: Change this to Xiaolin Wu’s line-drawing algorithm
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
  auto x0 = begin.x();
  auto y0 = begin.y();
  auto x1 = end.x();
  auto y1 = end.y();
  auto temp = x0;

  Eigen::Vector3f line_color = {255, 255, 255};
  Eigen::Vector3f point;

  bool steep = (abs(y1 - y0) > abs(x1 - x0));

  if(steep){
    temp = y0;
    y0 = x0;
    x0 = temp;
    temp = y1;
    y1 = x1;
    x1 = temp;
  }

  if(x0 > x1){
    temp = x1;
    x1 = x0;
    x0 = temp;
    temp = y1;
    y1 = y0;
    y0 = temp;
  }
  auto dx = x1 - x0;
  auto dy = y1 - y0;
  float gradient;
  if(static_cast<int>(dx) == 0){
    gradient = 1.0;
  }
  else{
    gradient = dy / dx;
  }
  auto xend = std::floor(x0 + 0.5);
  auto yend = y0 + gradient * (xend - x0);
  auto xgap = 1 - ((x0 + 0.5) - std::floor(x0 + 0.5));
  auto xpxl1 = xend;
  auto ypxl1 = std::floor(yend);

  if(steep){
    point = Eigen::Vector3f(ypxl1, xpxl1, 1.0f);
    set_pixel(point, line_color * ((1 - (yend - std::floor(yend))) * xgap));
    point = Eigen::Vector3f(ypxl1 + 1, xpxl1, 1.0f);
    set_pixel(point, line_color * ((yend - std::floor(yend)) * xgap));
  }
  else{
    point = Eigen::Vector3f(xpxl1, ypxl1, 1.0f);
    set_pixel(point, line_color * ((1 - (yend - std::floor(yend))) * xgap));
    point = Eigen::Vector3f(xpxl1, ypxl1 + 1, 1.0f);
    set_pixel(point, line_color * ((yend - std::floor(yend)) * xgap));
  }

  auto intery = yend + gradient;

  xend = std::floor(x1 + 0.5);
  yend = y1 + gradient * (xend - x1);
  xgap = (x1 + 0.5) - std::floor(x1 + 0.5);
  auto xpxl2 = xend;
  auto ypxl2 = std::floor(yend);
  if(steep){
    point = Eigen::Vector3f(ypxl2, xpxl2, 1.0f);
    set_pixel(point, line_color * ((1 - (yend - std::floor(yend))) * xgap));
    point = Eigen::Vector3f(ypxl2 + 1, xpxl2, 1.0f);
    set_pixel(point, line_color * ((yend - std::floor(yend)) * xgap));
  }
  else{
    point = Eigen::Vector3f(xpxl2, ypxl2, 1.0f);
    set_pixel(point, line_color * ((1 - (yend - std::floor(yend))) * xgap));
    point = Eigen::Vector3f(xpxl2, ypxl2 + 1, 1.0f);
    set_pixel(point, line_color * ((yend - std::floor(yend)) * xgap));
  }

  if(steep){
    for(int i = xpxl1 + 1; i < xpxl2; ++i){
      point = Eigen::Vector3f(std::floor(intery), i, 1.0f);
      set_pixel(point, line_color * (1 - (intery - std::floor(intery))));
      point = Eigen::Vector3f(std::floor(intery) + 1, i, 1.0f);
      set_pixel(point, line_color * (intery - std::floor(intery)));
      intery = intery + gradient;
    }
  }
  else{
    for(int i = xpxl1 + 1; i < xpxl2; ++i){
      point = Eigen::Vector3f(i, std::floor(intery), 1.0f);
      set_pixel(point, line_color * (1 - (intery - std::floor(intery))));
      point = Eigen::Vector3f(i, std::floor(intery) + 1, 1.0f);
      set_pixel(point, line_color * (intery - std::floor(intery)));
      intery = intery + gradient;
    }
  }

  /*
  // Use this function to draw a line from `begin` to `end` point on the image.
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

  if (dy1 <= dx1)
  {
    if (dx >= 0)
    {
      x = x1;
      y = y1;
      xe = x2;
    }
    else
    {
      x = x2;
      y = y2;
      xe = x1;
    }
    Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
    set_pixel(point, line_color);
    for (i = 0; x < xe; i++)
    {
      x = x + 1;
      if (px < 0)
      {
        px = px + 2 * dy1;
      }
      else
      {
        if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
        {
          y = y + 1;
        }
        else
        {
          y = y - 1;
        }
        px = px + 2 * (dy1 - dx1);
      }
      //            delay(0);
      Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
      set_pixel(point, line_color);
    }
  }
  else
  {
    if (dy >= 0)
    {
      x = x1;
      y = y1;
      ye = y2;
    }
    else
    {
      x = x2;
      y = y2;
      ye = y1;
    }
    Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
    set_pixel(point, line_color);
    for (i = 0; y < ye; i++)
    {
      y = y + 1;
      if (py <= 0)
      {
        py = py + 2 * dx1;
      }
      else
      {
        if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
        {
          x = x + 1;
        }
        else
        {
          x = x - 1;
        }
        py = py + 2 * (dx1 - dy1);
      }
      //            delay(0);
      Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
      set_pixel(point, line_color);
    }
  }*/
}

auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f)
{
  return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer,
                           rst::Primitive type)
{
  if (type != rst::Primitive::Triangle)
  {
    throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
  }
  auto &buf = pos_buf[pos_buffer.pos_id];
  auto &ind = ind_buf[ind_buffer.ind_id];

  float f1 = (100 - 0.1) / 2.0;
  float f2 = (100 + 0.1) / 2.0;
  Eigen::Matrix4f mvp = projection * view * model;
  for (auto &i : ind)
  {
    Triangle t;

    Eigen::Vector4f v[] = {mvp * to_vec4(buf[i[0]], 1.0f), mvp * to_vec4(buf[i[1]], 1.0f),
                           mvp * to_vec4(buf[i[2]], 1.0f)};
    for (auto &vec : v)
    {
      vec /= vec.w();
    }
    for (auto &vert : v)
    {
      vert.x() = 0.5 * width * (vert.x() + 1.0);
      vert.y() = 0.5 * height * (vert.y() + 1.0);
      vert.z() = vert.z() * f1 + f2;
    }
    for (int i = 0; i < 3; ++i)
    {
      t.setVertex(i, v[i].head<3>());
      t.setVertex(i, v[i].head<3>());
      t.setVertex(i, v[i].head<3>());
    }
    t.setColor(0, 255.0, 0.0, 0.0);
    t.setColor(1, 0.0, 255.0, 0.0);
    t.setColor(2, 0.0, 0.0, 255.0);
    rasterize_wireframe(t);
  }
}

void rst::rasterizer::rasterize_wireframe(const Triangle &t)
{
  draw_line(t.c(), t.a());
  draw_line(t.c(), t.b());
  draw_line(t.b(), t.a());
}

void rst::rasterizer::set_model(const Eigen::Matrix4f &m) { model = m; }

void rst::rasterizer::set_view(const Eigen::Matrix4f &v) { view = v; }

void rst::rasterizer::set_projection(const Eigen::Matrix4f &p) { projection = p; }

void rst::rasterizer::clear(rst::Buffers buff)
{
  if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
  {
    std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
  }
  if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
  {
    std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
  }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
  frame_buf.resize(w * h);
  depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y) { return (height - y) * width + x; }

void rst::rasterizer::set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color)
{
  // old index: auto ind = point.y() + point.x() * width;
  if (point.x() < 0 || point.x() >= width || point.y() < 0 || point.y() >= height)
    return;
  auto ind = (height - point.y()) * width + point.x();
  frame_buf[ind] = color;
}
