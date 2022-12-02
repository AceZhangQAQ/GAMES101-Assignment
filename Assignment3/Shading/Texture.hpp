//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v){
        if(u<0) u=0;
        if(v<0) v=0;
        if(u>1) u=1;
        if(v>1) v=1;

        //右上的点(向下取整)
        Eigen::Vector2f rt ((int) u*width,(int) (1-v)*height);
        //右下的点
        Eigen::Vector2f rb (rt.x(),rt.y()-1);
        //左上的点
        Eigen::Vector2f lt (rt.x()-1,rt.y());
        //左下的点
        Eigen::Vector2f lb (rt.x()-1,rt.y()-1);
        //四个点的颜色
        Eigen::Vector3f color_rt = getColor(rt.x()/width,rt.y()/height);
        Eigen::Vector3f color_rb = getColor(rb.x()/width,rb.y()/height);
        Eigen::Vector3f color_lt = getColor(lt.x()/width,lt.y()/height);
        Eigen::Vector3f color_lb = getColor(lb.x()/width,lb.y()/height);
        
        //采样点和左下角的点的距离差
        float s = u*width - lb.x();
        float t = (1-v)*height - lb.y();

        Eigen::Vector3f color_lb_rb = lerp(s,color_lb,color_rb);
        Eigen::Vector3f color_lt_rt = lerp(s,color_lt,color_rt);

        Eigen::Vector3f color = lerp(t,color_lb_rb,color_lt_rt);

        return color;
    }

    Eigen::Vector3f lerp(float x,Eigen::Vector3f color1,Eigen::Vector3f color2){
        Eigen::Vector3f color(0,0,0);
        //分别对RGB进行插值
        for (int i = 0; i < 3; i++){
            color[i] = color1[i] + x * (color2[i] - color1[i]);
        }
        return color;
    }

};
#endif //RASTERIZER_TEXTURE_H
