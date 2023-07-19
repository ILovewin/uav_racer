//
// Created by airvoltex on 22-8-16.
//

#ifndef MYUAV_BASIC_H
#define MYUAV_BASIC_H

//#define TIME_RECORD_IMAGE_PROCESS
#define SHOW_MASK_IMAGE
#define SHOW_DEPTH_IMAGE
//#define CONTROL_BY_PID

#include <cstdio>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <airsim_ros_pkgs/PoseCmd.h>

#include "airsim_ros_pkgs/VelCmd.h"

typedef enum {
    red,
    orange,
} RingColor;

typedef class {
public:
    cv::Mat src_rgb;
    cv::Mat src_depth;
    cv::Mat mask;
    enum {
        detected,
        not_detected,
    } detected_or_not;

    RingColor ring_color;

    //
    sensor_msgs::CameraInfo caminfo;

    //geometry_msgs::PointStamped是ROS中的消息类型
    geometry_msgs::PointStamped center_point;

    void detect();
    geometry_msgs::PointStamped get_ring_orthogonal();
    float depth_by_color_image_scale(int i, int j);
} VisionDetector;

typedef enum {
    is_in_the_position,
    not_in_the_position,
} Position_bool;

typedef enum {
    can_plan,
    cannot_plan,
} Planning_bool;

typedef volatile struct {
    float x;
    float y;
    float z;
} Point;


//前部摄像头图像的处理
void front_view_cb(const sensor_msgs::ImageConstPtr& msg);
//没懂这个转成深度图，打印出来有啥用
void front_depth_cb(const sensor_msgs::ImageConstPtr& msg);
void front_view_mask (const cv::Mat& frame, RingColor color);
//
void cam_info_cb(const sensor_msgs::CameraInfo& msg);
void real_pose_cb (nav_msgs::Odometry odem);
geometry_msgs::PointStamped detect(const cv::Mat& source,
                                   const cv::Mat& depth_source,
                                   const sensor_msgs::CameraInfo& camInfo
);
Position_bool control(geometry_msgs::Vector3 error, airsim_ros_pkgs::PoseCmd & p);
Planning_bool plan();


extern double proc_time; //ms
extern VisionDetector vision_detector;
extern geometry_msgs::TransformStamped data; // transform to publish
//extern nav_msgs::Odometry real_pose; //real_pose



#endif //MYUAV_BASIC_H
