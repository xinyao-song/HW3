#include <iostream>
#include <fstream>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "external/stb_image_write.h"

constexpr double MY_PI = 3.1415926;
const int image_w = 512, image_h = 512;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    double theta = eye_fov * MY_PI / 180;
    
    // Defining the Frustum
    double t = abs(zNear) * tan(theta / 2);
    double r = t * aspect_ratio;
    double l = -r;
    double b = -t;
    // Manually convert ZNear and zFar to negative.
    double n = -abs(zNear);
    double f = -abs(zFar);

    Eigen::Matrix4f perspToOrtho;
    perspToOrtho << n, 0, 0, 0, 
                    0, n, 0, 0, 
                    0, 0, n + f, -n * f, 
                    0, 0, 1, 0;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -(r + l) / 2, 
                0, 1, 0, -(t + b) / 2, 
                0, 0, 1, -(n + f) / 2, 
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2 / (r - l), 0, 0, 0, 
            0, 2 / (t - b), 0, 0, 
            0, 0, 2 / (n - f), 0, 
            0, 0, 0, 1;

    projection = scale * translate * perspToOrtho * projection;
    // std::cout << projection << std::endl << std::endl;
    return projection;
}


// DO NOT EDIT MAIN
int main(int argc, const char** argv)
{
    rst::rasterizer r(image_w, image_h);
    uint8_t *output_buf = new uint8_t[image_w * image_h * 3];

    Eigen::Vector3f eye_pos = {0,0,5};
    
    std::ifstream fin("testInput.txt");

    std::vector<Eigen::Vector3f> pos;
    std::vector<Eigen::Vector3i> ind;
    std::vector<Eigen::Vector3f> cols;


    // ####################### Input Vertices and Triangles Here #########################
    int numVertices, numTriangles;

    fin >> numVertices;

    for (int i = 0; i < numVertices; i++) {
        float x, y, z;
        fin >> x >> y >> z;
        pos.push_back(Eigen::Vector3f(x, y, z));

        float r, g, b;
        fin >> r >> g >> b;
        cols.push_back(Eigen::Vector3f(r, g, b));
    }

    fin >> numTriangles;

    for (int i = 0; i < numTriangles; i++) {
        int idx0, idx1, idx2;
        fin >> idx0 >> idx1 >> idx2;
        ind.push_back(Eigen::Vector3i(idx0, idx1, idx2));
    }
    
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    // ##########################################################################################

    float angle = 0.;

    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle)); // Angle is always 0 here
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50)); // Remember the ZNear and zFar here is positive. You need to manually convert them to negative.

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        std::string filename = "res.png";

        for (int i = 0; i < image_h; ++i) 
            for (int j = 0; j < image_w; ++j) {
            
                output_buf[(i * image_w + j) * 3] = (int)(r.frame_buffer()[i * image_w + j][0]);
                output_buf[(i * image_w + j) * 3 + 1] = (int)(r.frame_buffer()[i * image_w + j][1]);
                output_buf[(i * image_w + j) * 3 + 2] = (int)(r.frame_buffer()[i * image_w + j][2]);

            }

        stbi_write_png(filename.c_str(), image_w, image_h, 3, output_buf, image_w * 3);

    }

    return 0;
}
