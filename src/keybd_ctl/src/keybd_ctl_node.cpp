//
// Created by airvoltex on 22-8-16.
//
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "airsim_ros_pkgs/VelCmd.h"
#include "airsim_ros_pkgs/PoseCmd.h"
#include "airsim_ros_pkgs/Takeoff.h"
#include "airsim_ros_pkgs/Land.h"
#include "airsim_ros_pkgs/GPSYaw.h"
#include "nav_msgs/Odometry.h"
#include  "sensor_msgs/Imu.h"
#include <time.h>
#include <stdlib.h>


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

char kb=0;
bool exit_flag = 0;

int scanKeyboard()
{
    fd_set rfds;
    struct timeval tv;
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 10;
    if(select(1, &rfds, NULL, NULL, &tv)>0)
        return getchar();
    else
        return 0;
}

int main(int argc, char** argv)
{
    system("stty raw -echo -F" "/dev/tty"); //用于启动键盘监听
    ros::init(argc, argv, "keybd_ctl"); // 初始化ros 节点，命名为 basic
    ros::NodeHandle n; // 创建node控制句柄

    //通过这两个服务可以调用模拟器中的无人机起飞和降落命令
    ros::ServiceClient takeoff_client = n.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/takeoff");
    ros::ServiceClient land_client = n.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/drone_1/land");

    // 调用服务前需要定义特定的调用参数
    airsim_ros_pkgs::Takeoff takeoff;
    takeoff.request.waitOnLastTask = 1;
    airsim_ros_pkgs::Land land;
    land.request.waitOnLastTask = 1;

    //通过这两个publisher实现对无人机的速度控制和姿态控制
    ros::Publisher vel_publisher = n.advertise<airsim_ros_pkgs::VelCmd>("airsim_node/drone_1/vel_cmd_body_frame", 1);
    ros::Publisher pose_publisher = n.advertise<airsim_ros_pkgs::PoseCmd>("airsim_node/drone_1/pose_cmd_body_frame", 1);

    // 使用publisher发布速度指令需要定义 Velcmd , 并赋予相应的值后，将他publish（）出去
    airsim_ros_pkgs::VelCmd velcmd;
    velcmd.twist.angular.z = 0;//z方向角速度(yaw, deg)
    velcmd.twist.linear.x = 0.5; //x方向线速度(m/s)
    velcmd.twist.linear.y = 0.0;//y方向线速度(m/s)
    velcmd.twist.linear.z = -0.5;//z方向线速度(m/s)

    // 使用publisher发布速度指令需要定义 PoseCmd , 并赋予相应的值后，将他publish（）出去
    airsim_ros_pkgs::PoseCmd posecmd;
    posecmd.roll = 0; //x方向姿态(rad)
    posecmd.pitch = 0;//y方向姿态(rad)
    posecmd.yaw = 0;//z方向姿态(rad)
    posecmd.throttle = 0.596;//油门， （0.0-1.0）


    ros::Rate loop_rate(50);
    while(ros::ok()){

        //通过publish命令发布控制指令给模拟器
        // vel_publisher.publish(velcmd);
        // pose_publisher.publish(posecmd);

        //键盘控制无人机
        kb = scanKeyboard();
        switch (kb)
        {
            case '`':
                exit_flag = 1;
                break;
            case 't':
                // 通过 .call(参数) 的函数对模拟器中的服务发起起飞请求，程序暂停直到该服务返回结果
                if(takeoff_client.call(takeoff)){
                    ROS_INFO("Takeoff Succeed!");
                }else{
                    ROS_ERROR("Failed to takeoff, exit!");\
                return 0;
                }
                break;
            case 'l':
                // 通过 .call(参数) 的函数对模拟器中的服务发起降落请求，程序暂停直到该服务返回结果
                if(land_client.call(land)){
                    ROS_INFO("Land Succeed!");
                }else{
                    ROS_ERROR("Failed to land, exit!");\
                return 0;
                }
                break;
            case 'w':
                velcmd.twist.linear.y =0;
                velcmd.twist.angular.z =0;
                velcmd.twist.linear.z =0;
                velcmd.twist.linear.x += 0.1;
                if(velcmd.twist.linear.x>5) velcmd.twist.linear.x=5;
                break;
            case 's':
                velcmd.twist.linear.y =0;
                velcmd.twist.angular.z =0;
                velcmd.twist.linear.z =0;
                velcmd.twist.linear.x -= 0.1;
                if(velcmd.twist.linear.x<-5) velcmd.twist.linear.x=-5;
                break;
            case 'd':
                velcmd.twist.linear.x =0;
                velcmd.twist.angular.z =0;
                velcmd.twist.linear.z =0;
                velcmd.twist.linear.y += 0.1;
                if(velcmd.twist.linear.y>5) velcmd.twist.linear.y=5;
                break;
            case 'a':
                velcmd.twist.linear.x =0;
                velcmd.twist.angular.z =0;
                velcmd.twist.linear.z =0;
                velcmd.twist.linear.y -= 0.1;
                if(velcmd.twist.linear.y<-5) velcmd.twist.linear.y=-5;
                break;
            case 'e':
                velcmd.twist.linear.x =0;
                velcmd.twist.linear.y =0;
                velcmd.twist.linear.z =0;
                velcmd.twist.angular.z += 0.1;
                if(velcmd.twist.angular.z>1) velcmd.twist.angular.z=1;
                posecmd.roll =0;
                break;
            case 'q':
                velcmd.twist.linear.x =0;
                velcmd.twist.linear.y =0;
                velcmd.twist.linear.z =0;
                velcmd.twist.angular.z -= 0.1;
                if(velcmd.twist.angular.z<-1) velcmd.twist.angular.z=-1;
                posecmd.roll =0;
                break;
            case 'f':
                velcmd.twist.linear.x =0;
                velcmd.twist.linear.y =0;
                velcmd.twist.angular.z =0;
                velcmd.twist.linear.z += 0.1;
                if(velcmd.twist.linear.z>2) velcmd.twist.linear.z=2;
                break;
            case 'r':
                velcmd.twist.linear.x =0;
                velcmd.twist.linear.y =0;
                velcmd.twist.angular.z =0;
                velcmd.twist.linear.z -= 0.1;
                if(velcmd.twist.linear.z<-2) velcmd.twist.linear.z=-2;
                break;
            default:
                velcmd.twist.linear.x -=sgn<double>(velcmd.twist.linear.x)*0.1;
                velcmd.twist.linear.y -=sgn<double>(velcmd.twist.linear.y)*0.1;
                velcmd.twist.linear.z -=sgn<double>(velcmd.twist.linear.z)*0.1;
                velcmd.twist.angular.z -=sgn<double>(velcmd.twist.angular.z)*0.03;
                break;
        }
        if(exit_flag == 1) break;
        vel_publisher.publish(velcmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
    // stty -raw 不处理输入输出，（即使按ctrl + C也不会退出程序v）
    system("stty -raw echo -F" "/dev/tty");
    return 0;
}
