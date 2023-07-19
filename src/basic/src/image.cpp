//
// Created by airvoltex on 22-8-17.
//

#include "basic/basic.hpp"

double proc_time;//图像处理时间


void update_front_view_mask () {
#ifdef TIME_RECORD_IMAGE_PROCESS
    auto time_begin = ros::Time::now();
#endif

    // process in hsv color space
    cv::Mat hsv_frame;
    //使用cv::cvtColor函数，将名为vision_detector.src_rgb的RGB图像转换为HSV格式，并将结果保存在hsv_frame中
    cv::cvtColor(vision_detector.src_rgb, hsv_frame, cv::COLOR_BGR2HSV);

    //创建一个用于形态学转换的结构元素。使用cv::getStructuringElement函数，并指定形态学操作为开操作(cv::MORPH_OPEN)，结构元素的大小为5x5。创建的结构元素存储在element变量中，以供后续的形态学转换操作使用。开操作包括先腐蚀操作再膨胀操作，可以用来去除一些小的亮区域并放大局部低亮度的区域。
    cv::Mat element = cv::getStructuringElement(cv::MORPH_OPEN, cv::Size(5,5));

    if (vision_detector.ring_color == RingColor::orange) {
        // todo:调整参数
        //cv::inRange() 函数用于根据指定的颜色范围提取图像中的特定颜色区域（参数1：输入的 HSV 格式图像；参数2：颜色范围的下界；参数3：颜色范围的上界；参数4：输出的掩膜图像，将符合颜色范围的像素置为白色，不符合范围的像素置为黑色）
        cv::inRange(hsv_frame,  cv::Scalar(5, 150 ,150),
                    cv::Scalar(25, 255, 255), vision_detector.mask);

        //对上一步得到的掩膜图像vision_detector.mask进行形态学转换操作,实现去除噪点和放大局部低亮度区域的目的。
        cv::morphologyEx(vision_detector.mask, vision_detector.mask, cv::MORPH_OPEN, element);
    }
    else if(vision_detector.ring_color == RingColor::red){
        cv::Mat red_mask1, red_mask2;

        //todo:调整参数
        //根据指定的颜色范围提取图像中的特定颜色区域
        cv::inRange(hsv_frame, cv::Scalar(0, 150, 30), cv::Scalar(5, 255, 255), red_mask1);
        cv::inRange(hsv_frame, cv::Scalar(170, 150, 30), cv::Scalar(180, 255, 255), red_mask2);
        //对red_mask1和red_mask2做或运算，合并两个特征掩码,并结果存到vision_detector.mask
        cv::bitwise_or(red_mask1, red_mask2, vision_detector.mask);
//        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);

    }
#ifdef SHOW_MASK_IMAGE
    cv::imshow("front_view_mask", vision_detector.mask);// 显示前视图掩码图像
    cv::waitKey(10);
#endif

#ifdef TIME_RECORD_IMAGE_PROCESS
    auto time_spent = ros::Time::now() - time_begin;
    proc_time = time_spent.toSec()*1000;
#endif
}

void front_view_cb(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        //time record
        ros::Time time_begin = ros::Time::now();

        //cv_bridge是ROS提供的库，提供与openCV之间的接口,下面这个函数是将ROS传来的图像转为Mat的格式
        vision_detector.src_rgb = cv_bridge::toCvShare(msg, "bgr8")->image;
        //经测试发现，最后一个红色圆环中心的参考路径是准确的，所以统一将圆环颜色设置为橙色，对红色圆环无需做视觉处理
        vision_detector.ring_color = orange;
        update_front_view_mask();
    }
    catch (cv_bridge::Exception& e) {//捕获格式转换异常,并打印了无法将指定编码格式msg->encoding转换为bgr8格式的错误信息
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}



void front_depth_cb(const sensor_msgs::ImageConstPtr& msg) {
    try {//将接收到的深度图像转换为32位浮点数格式，并保存在vision_detector.src_depth中
        vision_detector.src_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
#ifdef SHOW_DEPTH_IMAGE
        cv::imshow("depth_view", vision_detector.src_depth);//显示深度图像
        cv::waitKey(10);
#endif
    }
    catch (cv_bridge::Exception& e) {//捕获cv_bridge异常，打印错误信息
        ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
    }
}