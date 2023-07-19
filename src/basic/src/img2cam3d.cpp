//
// Created by airvoltex on 22-8-21.
//
#include "basic/basic.hpp"

//geometry_msgs::TransformStamped 描述两个坐标系之间的关系：旋转矩阵R和平移矩阵T。
geometry_msgs::TransformStamped data;

void cam_info_cb(const sensor_msgs::CameraInfo& msg) {
    vision_detector.caminfo = msg;//将接收到的相机信息保存在 vision_detector.caminfo
    Point p;
    //todo:optimise this
    if(vision_detector.src_rgb.cols < 1) return;
}


//nav_msgs::Odometry是包含飞机位置和速度的数据结构
void real_pose_cb (nav_msgs::Odometry odem) {

    data.header.frame_id = "map";// 将数据头部的坐标系设置为 "map",表示数据中的位置和姿态信息是相对于 "map" 坐标系来描述的
    data.child_frame_id = "cam3d";//设置子坐标系为 "cam3d"
    data.header.stamp = ros::Time::now();

    //获取参数，将飞机的位置信息和姿态信息存入 data.transform 中
    data.transform.translation.x = odem.pose.pose.position.x;
    data.transform.translation.y = odem.pose.pose.position.y;
    data.transform.translation.z = odem.pose.pose.position.z;

    data.transform.rotation.w = odem.pose.pose.orientation.w;
    data.transform.rotation.x = odem.pose.pose.orientation.x;
    data.transform.rotation.y = odem.pose.pose.orientation.y;
    data.transform.rotation.z = odem.pose.pose.orientation.z;

}
