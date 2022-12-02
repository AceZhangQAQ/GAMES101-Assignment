#include "Triangle.hpp"
#include "rasterizer.hpp"
#include<cmath>
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    //获得x,y,z轴的单位向量
    Vector3f xAxis (1,0,0),yAxis (0,1,0),zAxis(0,0,1);
    //分解旋转轴在x,y,z轴上的投影
    float nx,ny,nz;
    nx = axis.x();
    ny = axis.y();
    nz = axis.z();

    //获得旋转角度
    float theta = angle/180.0f * M_PI;

    //构建罗德里格斯旋转矩阵3X3
    Matrix3f rodringuesRotation3f;
    //创建一个单位矩阵
    Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();
    //创建分量矩阵
    Eigen::Matrix3f N ;
    N << 0, -nz, ny,
         nz, 0, -nx,
         -ny, nx, 0;

    rodringuesRotation3f = cos(theta) * identity + (1-cos(theta)) * (axis * axis.transpose()) + sin(theta) * N;

    //构建罗德里格斯旋转矩阵4X4
    Matrix4f rodringuesRotation4f = Eigen::Matrix4f::Identity();

    //将3X3矩阵复制过去
    rodringuesRotation4f.block<3,3>(0,0) = rodringuesRotation3f;

    return rodringuesRotation4f;
}

Eigen::Matrix4f get_model_matrix(Vector3f axis,float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float theta = rotation_angle/180.0f * M_PI;

    //获得旋转矩阵
    Eigen::Matrix4f rotate = get_rotation(axis,rotation_angle);
    // rotate << cos(theta), -sin(theta), 0, 0,
    //           sin(theta),  cos(theta), 0, 0,
    //           0, 0, 1, 0,
    //           0, 0, 0, 1;
    
    //获得模型矩阵
    model = rotate * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    //根据eye_fov和aspect_ratio获取正交矩阵的left、right、top、botom
    float l, r, t, b, n=-zNear, f=-zFar;
    double theta = (eye_fov/2)/180*M_PI;
    t = tan(theta) * abs(zNear);
    b = -t;
    r = aspect_ratio * t;
    l = -r;

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    //获得透视投影到正交投影转换矩阵
    Eigen::Matrix4f M_Persp_To_Ortho;
    M_Persp_To_Ortho << n, 0, 0, 0,
                        0, n, 0, 0,
                        0, 0, n+f, -n*f,
                        0, 0, 1, 0;

    //初始化正交投影矩阵  
    Eigen::Matrix4f M_Ortho = Eigen::Matrix4f::Identity();
    //平移到原点
    Eigen::Matrix4f M_Ortho_Trans;
    M_Ortho_Trans << 1, 0, 0, -((r+l)/2),
                     0, 1, 0, -((t+b)/2),
                     0, 0, 1, -((n+f)/2),
                     0, 0, 0, 1;

    //单位化(缩放矩阵)
    Eigen::Matrix4f M_Ortho_Scale;
    M_Ortho_Scale << 2/(r-l), 0, 0, 0,
                     0, 2/(t-b), 0, 0,
                     0, 0, 2/(n-f), 0,
                     0, 0, 0, 1;
    
    //获得正交投影矩阵
    M_Ortho =  M_Ortho_Scale * M_Ortho_Trans * M_Ortho;

    //获得投影矩阵
    projection = M_Ortho * M_Persp_To_Ortho * projection;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    //旋转轴
    Eigen:: Vector3f axis = {0,1,0};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(axis,angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(axis,angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 3;
        }
        else if (key == 'd') {
            angle -= 3;
        }
    }

    return 0;
}
