
// clang-format off

#include <iostream>
#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <math.h>


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

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static float Direction(const Eigen::Vector2f &a, const Eigen::Vector2f &b, const Eigen::Vector2f &c) {
    Eigen::Vector2f AB = b - a;
    Eigen::Vector2f AC = c - a;
    return AB.x() * AC.y() - AB.y() * AC.x();
}


static bool insideTriangle(Vector2f &_p, const std::vector<Vector2f> &_v)
{   
    // #################################### TODO Q3 #######################################
    // TODO : Implement this function to check if the point _p is inside the triangle represented by _v[0], _v[1], _v[2] (screen space coordinates)
    float d1 = Direction(_v[0], _v[1], _p);
    float d2 = Direction(_v[1], _v[2], _p);
    float d3 = Direction(_v[2], _v[0], _p);

    if (((d1 < 0) && (d2 < 0) && (d3 < 0)) || ((d1 > 0) && (d2 > 0) && (d3 > 0))) {
        return true;
    } else {
        return false;
    }
}

std::vector<Eigen::Vector2f> rst::rasterizer::calculateSSCoords(const Vector3f* v_ndc)
{
    // ############################# TODO: Q2 ##################################
    // TODO : Calculate the triangle's screen space coordinates
    // The input is the coordinates of three vertices in NDC space
    // width: the screen width
    // height: the screen height

    std::vector<Eigen::Vector2f> ss_coords(3);

    for (int i = 0; i < 3; i++) {
        float x = (v_ndc[i].x() + 1) * width / 2.0;
        float y = (v_ndc[i].y() + 1) * height / 2.0;
        
        ss_coords[i] = Eigen::Vector2f(x, y);
    }

    return ss_coords;
}

static std::tuple<float, float, float> computeBarycentric2D(const std::vector<Vector2f> &v, float x, float y)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const std::vector<Vector2f> &v)
{
    return computeBarycentric2D(v, x, y);
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };

        // set depth
        for (int i = 0; i < 3; ++i)
        {
            t.setDepth(i, v[i].w());
        }

        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        for (int i = 0; i < 3; ++i)
        {
            t.setNDCVertex(i, v[i].head<3>());
            t.setNDCVertex(i, v[i].head<3>());
            t.setNDCVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    
    // #################################### TODO Q45 #######################################
    // In this function, you need to rasterize the triangle t into fragments.
    // There are some useful APIs you can use, the API usage will be mentioned below.
    //   t.v is the vertex position in the world space
    //   t.v_ndc is the vertex position after MVP transformation (which is in the NDC space). You can use t.v_ndc[i].z() to get the NDC depth of vertex i. 
    //   t.getColor() is the color of the triangle. Current the triangle contains a single color only.
    //   t.getDepth(i) is the depth value of vertex i before the perspective transformation. (which is the actual depth of the vertex and it is different from NDC depth).
    //  
    //   Use set_pixel to set the color of the pixel (i,j).
    //   Use set_depth to set the depth of the pixel (i,j).
    //   Use get_depth to get the depth of the pixel (i,j).
 
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    // you should implement and use the insideTriangle function above

    // Use the following code to get the interpolated z value. z value refers to the NDC depth. (x, y) is the current pixel position.

    //auto ss_coords = calculateSSCoords(t.v_ndc);
    //auto[alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, ss_coords);
    //float z_interpolated = alpha * t.v_ndc[0].z() + beta * t.v_ndc[1].z()  + gamma * t.v_ndc[2].z();

    //TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    // Calculate screen space coordinates of the triangle vertices
    auto ss_coords = calculateSSCoords(t.v_ndc);

    // Find bounding box of the triangle
    int x_min = std::min(ss_coords[0].x(), std::min(ss_coords[1].x(), ss_coords[2].x()));
    int x_max = std::max(ss_coords[0].x(), std::max(ss_coords[1].x(), ss_coords[2].x()));
    int y_min = std::min(ss_coords[0].y(), std::min(ss_coords[1].y(), ss_coords[2].y()));
    int y_max = std::max(ss_coords[0].y(), std::max(ss_coords[1].y(), ss_coords[2].y()));

    // Iterate through the bounding box
    for (int x = x_min; x <= x_max; x++) {
        for (int y = y_min; y <= y_max; y++) {

            Eigen::Vector2f p(x + 0.5, y + 0.5);
            if (insideTriangle(p, ss_coords)) {

                // Interpolate z value
                auto[alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, ss_coords);
                float z_interpolated = alpha * t.v_ndc[0].z() + beta * t.v_ndc[1].z()  + gamma * t.v_ndc[2].z();

                // Check if the pixel is closer than the current depth
                if (z_interpolated < get_depth(x, y)) {

                    // Set pixel color and depth
                    set_pixel(x, y, t.getColor());
                    set_depth(x, y, z_interpolated);

                }
            }
        }
    }

}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

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

int rst::rasterizer::get_index(int x, int y)
{
    return (height-y)*width + x;
}

void rst::rasterizer::set_pixel(int x, int y, const Eigen::Vector3f& color)
{
    auto ind = get_index(x, y);
    frame_buf[ind] = color * 255;
}

void rst::rasterizer::set_depth(int x, int y, float z)
{
    auto ind = get_index(x, y);
    depth_buf[ind] = z;
}

float rst::rasterizer::get_depth(int x, int y)
{
    auto ind = get_index(x, y);
    return depth_buf[ind];
}

// clang-format on
