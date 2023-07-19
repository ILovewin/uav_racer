//
// Created by airvoltex on 22-8-16.
//

#include "basic/basic.hpp"

VisionDetector vision_detector;

int main (int argc, char** argv) {
    ROS_INFO("start!\n");//向ROS系统的日志中输出一条信息。在这行代码中，输出的信息是 "start!\n"，表示程序开始执行的消息。

/// declare publishers
    ros::init(argc, argv, "basic"); // 初始化ros 节点，命名为 basic
    ros::NodeHandle n; // 创建node控制句柄
    image_transport::ImageTransport it(n); //创建图像传输控制句柄

    //创建一个名为 cam_info_suber 的订阅者对象。它订阅了名为 "airsim_node/drone_1/front_center/Scene/camera_info" 的话题，队列大小为1，并且指定了回调函数 cam_info_cb,通过回调函数将镜头传来的图片信息传给vision_detector.caminfo
    ros::Subscriber cam_info_suber = n.subscribe("airsim_node/drone_1/front_center/Scene/camera_info", 1, cam_info_cb);// camerainfo

    //创建名为 front_View_suber 的图像订阅者对象。它订阅了名为 "airsim_node/drone_1/front_center/Scene" 的图像话题，指定队列大小为1，并且指定了回调函数 front_view_cb ,通过回调函数将订阅得到无人机的坐标信息和速度信息,存到了data变量里面
    ros::Subscriber real_pose_suber = n.subscribe("airsim_node/drone_1/odom_local_ned", 1, real_pose_cb);

    //创建一个订阅者front_View_suber，用于接收"airsim_node/drone_1/front_center/Scene"话题中的图像消息。当有新的图像消息到达时，将调用名为front_view_cb的回调函数来对接收图像进行颜色提取和形态学转换的预处理操作
    image_transport::Subscriber front_View_suber = it.subscribe("airsim_node/drone_1/front_center/Scene", 1, front_view_cb);

    //创建一个订阅者front_depth_suber，用于接收"airsim_node/drone_1/front_center/DepthPlanar"话题中的深度图像消息。当有新的深度图像消息到达时，将调用名为front_depth_cb的回调函数把接收的深度图像消息储存在vision_detector.src_depth变量中
    image_transport::Subscriber front_depth_suber = it.subscribe("airsim_node/drone_1/front_center/DepthPlanar", 1, front_depth_cb);

//    ros::Publisher vel_publisher = n.advertise<airsim_ros_pkgs::VelCmd>("airsim_node/drone_1/vel_cmd_body_frame", 1);
//    ros::Publisher pose_publisher = n.advertise<airsim_ros_pkgs::PoseCmd>("airsim_node/drone_1/pose_cmd_body_frame", 1);

    //创建一个发布者detected_point_publisher，用于发布类型为geometry_msgs::PointStamped的经视觉检测处理的路径点消息到名为"detected_point"的话题上
    ros::Publisher detected_point_publisher = n.advertise<geometry_msgs::PointStamped>("detected_point", 1);
    //创建一个TF变换的广播器transform_puber，允许节点通过该广播器发布坐标系之间的变换关系，以便其他节点可以获取并应用这些变换
    tf2_ros::TransformBroadcaster transform_puber;
///

    vision_detector = VisionDetector();


    //ros::Rate 是 ROS 提供的一个类，用于控制循环的执行频率。这行代码创建了一个名为 loop_rate 的 Rate 对象，频率设置为 80 Hz，这意味着程序的主循环将以每秒 80 次的频率运行。
    ros::Rate loop_rate(80);
    while (ros::ok()) {//ros::ok() 是一个ROS提供的函数，用于检查ROS节点是否还在运行。在这个循环中，通过检查 ros::ok() 的返回值来确定是否继续循环执行。
#ifdef TIME_RECORD_IMAGE_PROCESS//如果定义了宏 TIME_RECORD_IMAGE_PROCESS，则编译下面的代码块。否则，这段代码块会被编译器忽略。
        ROS_INFO("process the image for %.2lf ms", proc_time);//通过定义或取消定义宏 TIME_RECORD_IMAGE_PROCESS，可以选择是否记录图像处理时间并输出日志
#endif

        //将二维深度图像坐标(像素坐标)转换为相机坐标系下的三维深度坐标
        vision_detector.detect();

        //标识未检测到圆环或者检测到的圆环位置不可信的情况
        if (vision_detector.detected_or_not == VisionDetector::not_detected
        ||  vision_detector.center_point.point.x < 0.5) {
            vision_detector.center_point.point.x = -1.0;
        }

        detected_point_publisher.publish(vision_detector.center_point);

        //在上面的real_pose_cb的回调函数里面已经给data赋值了
        transform_puber.sendTransform(data);

        ros::spinOnce();//处理回调函数和其他ROS消息
        loop_rate.sleep();//在每次循环迭代中，调用 loop_rate.sleep() 方法会使当前线程暂停一段时间，以达到实现目标循环频率的效果。具体的暂停时间由 loop_rate 对象计算并控制，以使整个循环运行时的平均频率接近设置的目标频率。
    }
    return 0;
}

