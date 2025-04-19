// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

bool SSAA = true;
//抗锯齿提高部分改自git佬
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


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f A = _v[0];
    Vector3f B = _v[1];
    Vector3f C = _v[2];

    Vector3f P(x,y,0);

    Vector3f AB = B - A;
    Vector3f BC = C - B;
    Vector3f CA = A - C;

    Vector3f AP = P - A;
    Vector3f BP = P - B;
    Vector3f CP = P - C;

    float V1 = AB.cross(AP).z();
    float V2 = BC.cross(BP).z();
    float V3 = CA.cross(CP).z();

    bool same;
    if((V1 >= 0 && V2 >= 0 && V3 >= 0) || (V1 <= 0 && V2 <= 0 && V3 <= 0))
    {
        same = true;
    }
    else
    {
        same = false;
    }
    return same;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);

    }
        if (SSAA)
        {
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < height; y++)
                {
                    Eigen::Vector3f color(0, 0, 0);
                    for (int i = 0; i < 4; i++)
                        color += frame_buf_2xSSAA[get_index(x, y)][i];
                    color /= 4;
                    set_pixel(Eigen::Vector3f(x, y, 1.0f), color);
                }
            }
        }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    //求取边界框；
    float Xmax =0;
    float Xmin = 0;
    for (int i = 0; i < 3; i++)
    {
        if (i == 0)
        {
            Xmax = v[i].x();
            Xmin = v[i].x();
        }
        else
        {
            if (v[i].x() > Xmax)
            {   
                Xmax = v[i].x();
            }
            else if (v[i].x() < Xmin)
            {
                Xmin = v[i].x();
            }
        }
    }

    float Ymax = v[0].y();
    float Ymin = v[0].y();
    for (int i = 0; i < 3; i++)
    {
        if (i == 0)
        {
            Ymax = v[i].y();
            Ymin = v[i].y();
        }
        else
        {
            if (v[i].y() > Ymax)
            {
                Ymax = v[i].y();
                
            }
            else if (v[i].y() < Ymin)
            {
                Ymin = v[i].y();
            }
        }
    }
  //来自git佬
    Ymax = std::floor(Ymax);
    Xmax = std::floor(Xmax);
    Ymin = std::ceil(Ymin);
    Xmin = std::ceil(Xmin);
    //ps：自己写完没加向下向上取整，导致遍历非整数值，对应像素位置出错

    //获取最大最小XY，得到包围盒；

    for (float i = Xmin; i <= Xmax; i++)
    {
        for (float j = Ymin; j <= Ymax; j++)
        {   
            float min_depth = FLT_MAX;
            if (SSAA)
            {
                int index = 0;
                for (float x = 0.25;x<=0.75;x+=0.5)
                {
                    for (float y = 0.25;y<=0.75;y+=0.5)
                    {
                        if (insideTriangle(i + x, j + y, t.v))
                        {
                            // 计算当前像素点在三角形内的重心坐标
                            std::tuple<float, float, float> barycentric = computeBarycentric2D(i+x, j+y, t.v);

                            float alpha = std::get<0>(barycentric);
                            float beta = std::get<1>(barycentric);
                            float gamma = std::get<2>(barycentric);

                            // 计算插值的 Z 值
                            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;

                            min_depth = std::min(min_depth, z_interpolated);
                            if (min_depth < depth_buf_2xSSAA[get_index(i, j)][index]) {
                                frame_buf_2xSSAA[get_index(i, j)][index] = t.getColor();
                                depth_buf_2xSSAA[get_index(i, j)][index] = min_depth;
                            }
                        }
                    index++;
                    }
                }
            }
            else
            {
                if (insideTriangle(i + 0.5, j + 0.5, t.v))
                {
                    // 计算当前像素点在三角形内的重心坐标
                    std::tuple<float, float, float> barycentric = computeBarycentric2D(i, j, t.v);

                    float alpha = std::get<0>(barycentric);
                    float beta = std::get<1>(barycentric);
                    float gamma = std::get<2>(barycentric);

                    // 计算插值的 Z 值
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    int posindex = get_index(i, j);

                    if (z_interpolated < depth_buf[posindex])
                    {
                        depth_buf[posindex] = z_interpolated;
                        set_pixel(Eigen::Vector3f(i, j, 0), t.getColor());
                    }

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
        //for循环遍历每一个像素的四个子像素的颜色值
        for (int i = 0; i < frame_buf_2xSSAA.size(); i++) {
            frame_buf_2xSSAA[i].resize(4);
            std::fill(frame_buf_2xSSAA[i].begin(), frame_buf_2xSSAA[i].end(), Eigen::Vector3f{ 0, 0, 0 });
        }
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        //for循环遍历每一个像素的四个子像素的颜色值
        for (int i = 0; i < depth_buf_2xSSAA.size(); i++) {
            depth_buf_2xSSAA[i].resize(4);
            std::fill(depth_buf_2xSSAA[i].begin(), depth_buf_2xSSAA[i].end(), std::numeric_limits<float>::infinity());
        }
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    //2xSSAA的缓存为二维数组
    frame_buf_2xSSAA.resize(w * h);
    depth_buf_2xSSAA.resize(w * h);

}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on