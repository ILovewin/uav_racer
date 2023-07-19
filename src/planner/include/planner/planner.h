//
// Created by airvoltex on 22-8-29.
//

#ifndef PLANNER_PLANNER_H
#define PLANNER_PLANNER_H

#include <cstdio>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
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
#include <airsim_ros_pkgs/VelCmd.h>



typedef class {
public:
    geometry_msgs::PointStamped point;
    enum {
        vision,
        fixed,
    } source;

    enum {
        reliable,
        not_reliable,
    } is_reliable;

    enum {
        virtual_center,
        ring_center,
    } is_ring_center;

    enum {
        no_reaching,
        approaching,
        passed,
    } status;

    float get_distance_square(geometry_msgs::PointStamped p);
} WayPoint;


#endif //PLANNER_PLANNER_H
