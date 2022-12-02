// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float theta = rotation_angle/180.0f * M_PI;

    //获得旋转矩阵
    Eigen::Matrix4f rotate;
    rotate << cos(theta), -sin(theta), 0, 0,
              sin(theta),  cos(theta), 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    
    //获得模型矩阵
    model = rotate * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    //根据eye_fov和aspect_ratio获取正交矩阵的left、right、top、botom
    float l, r, t, b;
    double theta = (eye_fov/2)/180*M_PI;
    t = tan(theta) * abs(zNear);
    b = -t;
    r = aspect_ratio * t;
    l = -r;

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    //获得透视投影到正交投影转换矩阵
    Eigen::Matrix4f M_Persp_To_Ortho;
    M_Persp_To_Ortho << -zNear, 0, 0, 0,
                        0, -zNear, 0, 0,
                        0, 0, -zNear-zFar, -zNear*zFar,
                        0, 0, 1, 0;

    //初始化正交投影矩阵  
    Eigen::Matrix4f M_Ortho = Eigen::Matrix4f::Identity();
    //平移到原点
    Eigen::Matrix4f M_Ortho_Trans;
    M_Ortho_Trans << 1, 0, 0, -((r+l)/2),
                     0, 1, 0, -((t+b)/2),
                     0, 0, 1, -((-zNear-zFar)/2),
                     0, 0, 0, 1;

    //单位化(缩放矩阵)
    Eigen::Matrix4f M_Ortho_Scale;
    M_Ortho_Scale << 2/(r-l), 0, 0, 0,
                     0, 2/(t-b), 0, 0,
                     0, 0, 2/(-zNear+zFar), 0,
                     0, 0, 0, 1;
    
    //获得正交投影矩阵
    M_Ortho = M_Ortho_Scale * M_Ortho_Trans *  M_Ortho;

    //获得投影矩阵
    projection = M_Ortho * M_Persp_To_Ortho * projection;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = true;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0,0,5};

    //旋转轴
    Eigen:: Vector3f axis = {0,0,1};

    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(0.1);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
// clang-format on