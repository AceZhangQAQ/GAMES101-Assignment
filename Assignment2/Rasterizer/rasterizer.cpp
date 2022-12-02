// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
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


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    //定义出目标点
    Eigen:Vector3f point (x,y,1.0f);

    //定义出由三角形顶点指向目标点的三个向量
    Eigen::Vector3f to_point_vector1 = point - _v[0];
    Eigen::Vector3f to_point_vector2 = point - _v[1];
    Eigen::Vector3f to_point_vector3 = point - _v[2];

    //定义出三角形三条边向量
    Eigen::Vector3f triangleEdge1 = _v[1] - _v[0];
    Eigen::Vector3f triangleEdge2 = _v[2] - _v[1];
    Eigen::Vector3f triangleEdge3 = _v[0] - _v[2];

    float z1 = triangleEdge1.cross(to_point_vector1).z();
    float z2 = triangleEdge2.cross(to_point_vector2).z();
    float z3 = triangleEdge3.cross(to_point_vector3).z();

    return (z1 > 0 && z2 > 0 && z3 > 0) || (z1 < 0 && z2 < 0 && z3 < 0);

    //利用叉乘的结果判断目标点是否在三角形内部
    // Eigen::Vector3f cross_result1 = triangleEdge1.cross(to_point_vector1);
    // Eigen::Vector3f cross_result2 = triangleEdge2.cross(to_point_vector2);
    // Eigen::Vector3f cross_result3 = triangleEdge3.cross(to_point_vector3);

    //如果向量方向相同则点乘结果同号
    // if((cross_result1.dot(cross_result2) > 0 && cross_result1.dot(cross_result3) >0) || (cross_result1.dot(cross_result2) < 0 && cross_result1.dot(cross_result3) < 0))
    //     return true;
    // else
    //     return false;
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
            vert.z() = -(vert.z() * f1 + f2);
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
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    //是否开启MSAA
    bool msaa_on = true;
    //一个像素内的四个基采样点
    float base_sample_point [][2] = {{0.25,0.25},{0.25,0.75},{0.75,0.25},{0.75,0.75}};

    // //初始化包围盒
    int left = 10000, right = 0, top = 0, bottom = 10000;
    //遍历三角形的三个顶点，找到包围盒
    for (int i = 0; i < 3; i++){
        //如果此点x坐标小于left则更新
        if(v[i].x() < left)
            left = v[i].x();
        //如果此点x坐标大于right则更新
        if(v[i].x() > right)
            right = v[i].x();
        //如果此点y坐标小于bottom则更新
        if(v[i].y() < bottom)
            bottom = v[i].y();
        //如果此点y坐标大于top则更新
        if(v[i].y() > top)
            top = v[i].y();
    }

    //遍历包围盒内的每个像素，找到在三角形中的点
    for (int x = left; x <= right; x++){
        for (int y = bottom; y <= top; y++){
            //判断此像素点是否在三角形内部
            //4X MSAA采样 判断每个像素点内的四个采样点是否在三角形内部
            if(msaa_on){
                //初始化此像素点上色比率
                int color_rate = 0;
                //MSAA 4X 采样
                for(int m = 0; m < 4; m++){
                    if(insideTriangle(x+base_sample_point[m][0],y+base_sample_point[m][1],t.v)){
                        auto[alpha, beta, gamma] = computeBarycentric2D(x+base_sample_point[m][0],y+base_sample_point[m][1], t.v);
                        float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        //如果此点的插值深度值小于深度缓冲区深度值
                        if(z_interpolated < depth_sample[get_4X_sample_index(x*2 + m % 2, y*2 + m / 2)]){
                            //更新此像素点深度缓存值
                            depth_sample[get_4X_sample_index(x*2 + m % 2, y*2 + m / 2)] = z_interpolated;
                            //设置颜色
                            set_4X_sample_pixel(Eigen::Vector3f(x*2 + m % 2, y*2 + m / 2,1.0f),t.getColor());
                            color_rate += 1;
                        } 
                    }
                }
                //对超采样结果进行上色
                if(color_rate > 0){
                    Eigen::Vector3f point_color = (frame_sample[get_4X_sample_index(x*2, y*2)] 
                                                  +frame_sample[get_4X_sample_index(x*2+1, y*2)]
                                                  +frame_sample[get_4X_sample_index(x*2, y*2+1)]
                                                  +frame_sample[get_4X_sample_index(x*2+1, y*2+1)]) / 4;
                    set_pixel(Eigen::Vector3f(x,y,1.0f),point_color);
                }
            }else{
                if(insideTriangle(x+0.5f,y+0.5f,t.v)){
                    auto[alpha, beta, gamma] = computeBarycentric2D(x+0.5f, y+0.5f, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    //如果此点的插值深度值小于深度缓冲区深度值
                    if(z_interpolated < depth_buf[get_index(x,y)]){
                        //更新此像素点深度缓存值
                        depth_buf[get_index(x,y)] = z_interpolated;
                        //设置颜色
                        set_pixel(Eigen::Vector3f(x,y,z_interpolated),t.getColor());
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
        std::fill(frame_sample.begin(), frame_sample.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_sample.begin(), depth_sample.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    frame_sample.resize(w * h * 4);
    depth_sample.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_4X_sample_index(int x, int y)
{
    return (height*2-1-y)*width*2 + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;
}

void rst::rasterizer::set_4X_sample_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height*2-1-point.y())*width*2 + point.x();
    frame_sample[ind] = color;
}

// clang-format on