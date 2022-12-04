#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    //如果控制点序列大小为1，则直接返回该点
    if(control_points.size() == 1){
        return control_points[0];
    }
    //否则继续递归
    else{
        //创建一个原控制点长度-1的容器，用来存储新生成的控制点
        std::vector<cv::Point2f> new_control_points;
        //遍历原控制点序列
        for (size_t i = 0; i < control_points.size() - 1; i++){
            //根据前后两个控制点生成新的一个控制点
            cv::Point2f new_point = (1-t)*control_points[i] + t*control_points[i+1];
            //将新生成的控制点加入序列中
            new_control_points.push_back(new_point);
        }
        //用新生成的控制点序列继续递归
        return recursive_bezier(new_control_points,t);
    }
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t= 0.0; t <= 1.0; t+= 0.001){
        auto point = recursive_bezier(control_points,t);
        //根据到像素中心的距离对 Bézier 曲线反走样
        //考虑九个像素点的着色
        cv::Point2f pixel_base ((int)point.x,(int)point.y);//九个像素点基准位置
        cv::Point2f pixel_t_1 (pixel_base.x-0.5f,pixel_base.y+1.5f);//上1的像素点
        cv::Point2f pixel_t_2 (pixel_base.x+0.5f,pixel_base.y+1.5f);//上2的像素点
        cv::Point2f pixel_t_3 (pixel_base.x+1.5f,pixel_base.y+1.5f);//上3的像素点
        cv::Point2f pixel_m_1 (pixel_base.x-0.5f,pixel_base.y+0.5f);//上1的像素点
        cv::Point2f pixel_m_2 (pixel_base.x+0.5f,pixel_base.y+0.5f);//上2的像素点
        cv::Point2f pixel_m_3 (pixel_base.x+1.5f,pixel_base.y+0.5f);//上3的像素点
        cv::Point2f pixel_b_1 (pixel_base.x-0.5f,pixel_base.y-0.5f);//下1的像素点
        cv::Point2f pixel_b_2 (pixel_base.x+0.5f,pixel_base.y-0.5f);//上2的像素点
        cv::Point2f pixel_b_3 (pixel_base.x+1.5f,pixel_base.y-0.5f);//上3的像素点

        //此点到九个像素点的距离
        float d_t_1 = cv::norm(pixel_t_1 - point);
        float d_t_2 = cv::norm(pixel_t_2 - point);
        float d_t_3 = cv::norm(pixel_t_3 - point);
        float d_m_1 = cv::norm(pixel_m_1 - point);
        float d_m_2 = cv::norm(pixel_m_2 - point);
        float d_m_3 = cv::norm(pixel_m_3 - point);
        float d_b_1 = cv::norm(pixel_b_1 - point);
        float d_b_2 = cv::norm(pixel_b_2 - point);
        float d_b_3 = cv::norm(pixel_b_3 - point);

        //根据距离信息着色
        //中间的点距离范围(0-sqrt(2)/2)
        window.at<cv::Vec3b>(pixel_m_2.y,pixel_m_2.x)[1] = std::fmax(window.at<cv::Vec3b>(pixel_m_2.y,pixel_m_2.x)[1],255 * (1-d_m_2/(sqrt(2)/2)));
        //上下左右四点的距离范围(0-sqrt(10)/2)
        window.at<cv::Vec3b>(pixel_t_2.y,pixel_t_2.x)[1] = std::fmax(window.at<cv::Vec3b>(pixel_t_2.y,pixel_t_2.x)[1],255 * (1-d_t_2/(sqrt(10)/2)));
        window.at<cv::Vec3b>(pixel_m_1.y,pixel_m_1.x)[1] = std::fmax(window.at<cv::Vec3b>(pixel_m_1.y,pixel_m_1.x)[1],255 * (1-d_m_1/(sqrt(10)/2)));
        window.at<cv::Vec3b>(pixel_m_3.y,pixel_m_3.x)[1] = std::fmax(window.at<cv::Vec3b>(pixel_m_3.y,pixel_m_3.x)[1],255 * (1-d_m_3/(sqrt(10)/2)));
        window.at<cv::Vec3b>(pixel_b_2.y,pixel_b_2.x)[1] = std::fmax(window.at<cv::Vec3b>(pixel_b_2.y,pixel_b_2.x)[1],255 * (1-d_b_2/(sqrt(10)/2)));
        //四个角的点距离范围(0-3*sqrt(2)/2)
        window.at<cv::Vec3b>(pixel_t_1.y,pixel_t_1.x)[1] = std::fmax(window.at<cv::Vec3b>(pixel_t_1.y,pixel_t_1.x)[1],255 * (1-d_t_1/(3*sqrt(2)/2)));
        window.at<cv::Vec3b>(pixel_t_3.y,pixel_t_3.x)[1] = std::fmax(window.at<cv::Vec3b>(pixel_t_3.y,pixel_t_3.x)[1],255 * (1-d_t_3/(3*sqrt(2)/2)));
        window.at<cv::Vec3b>(pixel_m_1.y,pixel_m_1.x)[1] = std::fmax(window.at<cv::Vec3b>(pixel_m_1.y,pixel_m_1.x)[1],255 * (1-d_m_1/(3*sqrt(2)/2)));
        window.at<cv::Vec3b>(pixel_m_3.y,pixel_m_3.x)[1] = std::fmax(window.at<cv::Vec3b>(pixel_m_3.y,pixel_m_3.x)[1],255 * (1-d_m_3/(3*sqrt(2)/2)));
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }
        //硬编码四个点，方便对比效果
        // std::vector<cv::Point2f> hard_code_points(4);
        // hard_code_points[0].x = 192;
        // hard_code_points[0].y = 206;
        // cv::circle(window, hard_code_points[0], 3, {255, 255, 255}, 3);
        // hard_code_points[1].x = 478;
        // hard_code_points[1].y = 159;
        // cv::circle(window, hard_code_points[1], 3, {255, 255, 255}, 3);
        // hard_code_points[2].x = 554;
        // hard_code_points[2].y = 395;
        // cv::circle(window, hard_code_points[2], 3, {255, 255, 255}, 3);
        // hard_code_points[3].x = 293;
        // hard_code_points[3].y = 492;
        // cv::circle(window, hard_code_points[3], 3, {255, 255, 255}, 3);
        
        if (control_points.size() == 4) 
        {
            // naive_bezier(control_points, window);
            
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
