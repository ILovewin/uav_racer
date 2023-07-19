//
// Created by airvoltex on 22-8-29.
//


#include "planner/planner.h"
#include "math.h"

#define WAY_POINT_COUNT 8
#define Kp 1
#define Kp_YAW 1.2
#define MAX_SPEED 10.0*Kp
#define MIN_SPEED  8.0*Kp
#define YAW_TRACK
#define STRAIGHT_SPEED_UP 1.2
//#define CONTROL_POLYNOMIAL
#define CONTROL_PID

geometry_msgs::PointStamped detected_point;

//nav_msgs::Odometry类型包含了机器人在空间中的速度和位置信息
nav_msgs::Odometry real_pose;


float WayPoint::get_distance_square(geometry_msgs::PointStamped p) {

    return pow(this->point.point.x - p.point.x, 2) +
            pow(this->point.point.y - p.point.y, 2) +
            pow(this->point.point.z - p.point.z, 2);
}

void init_reference_waypoint (WayPoint* way_point_set) {
    ROS_INFO("Initialize reference point");
    //除了下标为4的那组数据，其它均为官方提供的障碍环的近似坐标,增加一组数据是为了优化路径
    float goalpath[WAY_POINT_COUNT][3] = {{14.8715, -0.7906, -2.6},
                                          {37.68, -12.27, -1.02},
                                          {67.42, -12.94, -0.30},
                                          {94.44, -8.05, 0.02},
                                          {95, -10, 0},
                                          {113.95, -35.70, -0.34},
                                          {121.77, -67.73, -3.83},
                                          {121.80, -96, -7.39}};

    for (int i =0; i<WAY_POINT_COUNT; i++) {
        // Set header frame ID and point coordinates for each waypoint
        way_point_set[i].point.header.frame_id = "map";
        way_point_set[i].point.point.x = goalpath[i][0];
        way_point_set[i].point.point.y = goalpath[i][1];
        way_point_set[i].point.point.z = goalpath[i][2];
        // Set other properties of the waypoint
        way_point_set[i].source = WayPoint::fixed;
        way_point_set[i].is_reliable = WayPoint::not_reliable;
        way_point_set[i].is_ring_center = WayPoint::ring_center;
        way_point_set[i].status = WayPoint::no_reaching;
    }
}



void process_detected_point(WayPoint* reference_point, geometry_msgs::PointStamped p, tf2_ros::Buffer& buffer) {

    if (p.point.x < 1) {
        // trait as not detected
        return;
    }

    // get closest point
    // first for index & second for distance(distance square in processing)
    std::pair<int, float> closest_point(0, 99999999999);
    try {
        p = buffer.transform(p, "map");//Transform the detected point to the "map" frame
    }
    catch (std::exception &e) {
        ROS_WARN("%s", e.what()); //Log a warning if transformation fails
        return;
    }
    for (int i = 0;i<WAY_POINT_COUNT; i++) {
        //计算参考路径点坐标与视觉检测点坐标之间的距离distance_square
        float distance_square = reference_point[i].get_distance_square(p);

        if (distance_square < closest_point.second) {
            closest_point.second = distance_square;
            closest_point.first = i;
        }
    }

    if (closest_point.second > 10000) {
        return;
    }

    //Update the reference point with the closest detected point
    int update_index = closest_point.first;
    ROS_INFO("updated %d waypoint", update_index);
    reference_point[update_index].point = p;
    reference_point[update_index].is_reliable = WayPoint::reliable;
    reference_point[update_index].is_ring_center = WayPoint::ring_center;
    reference_point[update_index].source = WayPoint::vision;
}

//geometry_msgs::PointStamped 是一个 ROS 消息类型，用于表示带有参考坐标系和时间戳的点
void detected_point_cb(geometry_msgs::PointStamped msg) {
    detected_point = msg;
    detected_point.header.frame_id = "cam3d";
}


airsim_ros_pkgs::VelCmd control_pid(float x, float y, float z, bool straight) {
    static float Ix = 0;
    static float Iy = 0;
    static float Iz = 0;
    airsim_ros_pkgs::VelCmd vel_cmd;

    float vx = Kp * x;
    float vy = Kp * y;
    float vz = Kp * z;

    // 计算速度矢量的模
    auto v = (float)sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));

    // 将速度限制在最大和最小速度范围内
    if (v > MAX_SPEED) {
        vx = vx / v * MAX_SPEED;
        vy = vy / v * MAX_SPEED;
        vz = vz / v * MAX_SPEED;
    } else if (v < MIN_SPEED) {
        vx = vx / v * MIN_SPEED;
        vy = vy / v * MIN_SPEED;
        vz = vz / v * MIN_SPEED;
    }

    // 根据直线运动对速度进行缩放
    vx *= STRAIGHT_SPEED_UP;
    vy *= STRAIGHT_SPEED_UP;
    vz *= STRAIGHT_SPEED_UP;

    //由于无人机在第四个障碍环到第五个障碍环处的路径与其它部分路径差异较大，故针对路径的特定部分进行优化
    if (straight){
        vy *= 1.6;
        vz *= 1.8;
    }
    else {
        vy *= 1.2;
        vz *= 1.8;
    }
    // 设置线性速度到速度命令
    vel_cmd.twist.linear.x = vx;
    vel_cmd.twist.linear.y = vy;
    vel_cmd.twist.linear.z = vz;

#ifdef YAW_TRACK
    // 计算用于航向跟踪的期望偏航角速度
    vel_cmd.twist.angular.z = Kp_YAW * atan(vy/vx);
#endif
    return vel_cmd;
}

airsim_ros_pkgs::VelCmd update_vel(WayPoint* reference_point, tf2_ros::Buffer& buffer, airsim_ros_pkgs::VelCmd vel_last) {
    for (int i =0; i<WAY_POINT_COUNT; i++) {
        if (reference_point[i].status == WayPoint::passed) {//跳过已经经过的路径点
            continue;
        }
        geometry_msgs::PointStamped cam_point;
        try{
            reference_point[i].point.header.stamp = ros::Time::now() - ros::Duration(0.1);//将 reference_point[i].point.header.stamp 的值设置为当前时间减去 0.1 秒
            cam_point = buffer.transform(reference_point[i].point, "cam3d");// Transform the reference point from "gnd3d" frame to "cam3d" frame
        }
        catch (std::exception &e) {
            ROS_INFO("%s", e.what());
            return vel_last;
        }

        // 计算转换后的路径点与原点(相机位置)的平方距离
        float distance_square = pow(cam_point.point.x, 2) +
                pow(cam_point.point.y, 2) +
                pow(cam_point.point.z, 2);
        if (reference_point[i].status == WayPoint::approaching || reference_point[i].status == WayPoint::no_reaching) {//检查参考路径点的状态，如果无人机还未经过或者正在经过此参考路径点，打印此路径点与无人机之间的距离信息
            ROS_INFO("Approaching point distance %.2lf", sqrt(distance_square));
            if (cam_point.point.x < 0) {//x的坐标值为深度相机测得的无人机到圆环中心的距离，所以当x<0时，更新此路径点状态为pass
                reference_point[i].status = WayPoint::passed;
            }
            else{//所以当x>0时，更新此路径点状态为approaching
                reference_point[i].status = WayPoint::approaching;
            }
            if (distance_square < 16) {//当转换后的路径点与原点的平方距离小于16时返回上一次的速度
                return vel_last;
            }
            airsim_ros_pkgs::VelCmd result;
#ifdef CONTROL_POLYNOMIAL
#ifdef CONTROL_PID
            ROS_ERROR("multiple define in CONTROL!!!!")

#endif
            static ros::Time planned_time = ros::Time(0);
            static int signalcount = 15;

//                result.twist.angular.z = control_pid(cam_point.point.x,
//                                                     cam_point.point.y,
//                                                     reference_point[i].point.point.z - real_pose.pose.pose.position.z,
//                                                     (i!=5)).twist.angular.z;
            // replan
            // make points matrix(time and pts)
            int num_points = WAY_POINT_COUNT - i+1;
            Eigen::MatrixXd pos(3, num_points); // include current position and remaining waypoints
            pos(0,0) = real_pose.pose.pose.position.x;
            pos(1,0) = real_pose.pose.pose.position.y;
            pos(2,0) = real_pose.pose.pose.position.z;
            for (int j = 1;j<num_points;j++) {
                pos(0,j) = reference_point->point.point.x;
                pos(1,j) = reference_point->point.point.y;
                pos(2,j) = reference_point->point.point.z;
            }
            // write time matrix
            Eigen::VectorXd time(num_points-1);
            geometry_msgs::PointStamped current_point;
            current_point.point.x= real_pose.pose.pose.position.x;
            current_point.point.y= real_pose.pose.pose.position.y;
            current_point.point.z= real_pose.pose.pose.position.z;
            auto distance = sqrt(reference_point[i].get_distance_square(current_point));
            time(0) = distance / MAX_SPEED;
            for (int j =1;j<num_points-1;j++ ) { // excuse me for the j (((
                time(j) = sqrt(reference_point[i+j].get_distance_square(reference_point[i+j-1].point)) / MAX_SPEED;
            }

            if (pos.cols() >= 3) {
                gl_traj = PolynomialTraj::minSnapTraj(pos,
                                                      Eigen::Vector3d(real_pose.twist.twist.linear.x,
                                                                      real_pose.twist.twist.linear.y,
                                                                      real_pose.twist.twist.linear.z),
                                                      Eigen::Vector3d(MAX_SPEED, 0, 0),
                                                      Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), time);
            }
            else if (pos.cols() == 2) {
                gl_traj = PolynomialTraj::one_segment_traj_gen(pos.col(0),
                                                               Eigen::Vector3d(real_pose.twist.twist.linear.x,
                                                                               real_pose.twist.twist.linear.y,
                                                                               real_pose.twist.twist.linear.z),
                                                               Eigen::Vector3d(0, 0, 0), pos.col(1),
                                                               Eigen::Vector3d(MAX_SPEED, 0, 0),
                                                               Eigen::Vector3d(0, 0, 0), time(0));
            }
            Eigen::Vector3d vel = gl_traj.evaluateVel(0.2);
            result.twist.linear.x = vel.x();
            result.twist.linear.y = vel.y();
            result.twist.linear.z = vel.z();
#endif
#ifdef CONTROL_PID
            //调用自定义的control_pid函数对速度做pid控制
            result = control_pid(cam_point.point.x,
                               cam_point.point.y,
                               reference_point[i].point.point.z - real_pose.pose.pose.position.z,
//                                    cam_point.point.z,
                               (i!=5));
//                               true);
#endif

            return result;
        }
    }
}


//nav_msgs::Odometry 是一个 ROS 消息类型，用于存储机器人在自由空间中的位置和速度估计
void real_pose_cb (nav_msgs::Odometry odem) {
    real_pose = odem;//将odem赋值给全局变量real_pose，以更新real_pose的值为最新的里程计信息
}

int main (int argc, char** argv) {
    ROS_INFO("Planner start!");//向ROS系统的日志中输出一条信息。在这行代码中，输出的信息是 "Planner start!"，表示飞机起飞的消息。
    ros::init(argc, argv, "planner");// 初始化ros 节点，命名为 planner
    ros::NodeHandle n;// 创建node控制句柄

    //tf2 消息是指 tf2 库中用于传输转换数据的消息。它们通常由 tf2_ros::TransformBroadcaster 类的 sendTransform 方法发送，并由 tf2_ros::TransformListener 类订阅
    tf2_ros::Buffer buffer;//创建一个名为 buffer 的tf2_ros::Buffer对象，用于存储和管理坐标变换的缓冲区
    tf2_ros::TransformListener listener(buffer);//创建一个tf2_ros::TransformListener对象 listener，并将之前创建的 buffer 作为参数传递给它。这个监听器会订阅并接收tf2坐标变换的消息，并将其存储在 buffer 中

    //创建一个名为 detected_point_suber 的ROS消息订阅者，它是订阅类型为 geometry_msgs::PointStamped 的名为 "detected_point" 的消息(是经视觉处理得到的点信息)。当有新的消息到达时，将调用回调函数 detected_point_cb 进行处理
    ros::Subscriber detected_point_suber = n.subscribe<geometry_msgs::PointStamped>("detected_point", 1, detected_point_cb);

    //创建一个名为 real_pose_suber 的ROS消息订阅者，它是订阅名为 "airsim_node/drone_1/odom_local_ned" 的消息。当有新的消息到达时，将调用回调函数 real_pose_cb 更新全局变量real_pose中存储的位置和速度估计
    ros::Subscriber real_pose_suber = n.subscribe("airsim_node/drone_1/odom_local_ned", 1, real_pose_cb);

    //创建一个名为 vel_publisher 的ROS消息发布者，它是发布类型为 airsim_ros_pkgs::VelCmd 的消息到名为 "airsim_node/drone_1/vel_cmd_body_frame" 的话题上。通过这个发布者，可以将速度命令消息发送到相应的话题上，以控制无人机的运动。
    ros::Publisher vel_publisher = n.advertise<airsim_ros_pkgs::VelCmd>("airsim_node/drone_1/vel_cmd_body_frame", 1);

    //用于调试
    ros::Publisher map_position_publisher = n.advertise<geometry_msgs::PointStamped>("real_position", 1);

    auto reference_point = new WayPoint[WAY_POINT_COUNT];//创建一个动态大小的 WayPoint 数组

    //初始化参考路径点，将官方提供的参考坐标值赋给 reference_point 数组中的每个元素
    init_reference_waypoint(reference_point);

    //ros::Rate 是 ROS 提供的一个类，用于控制循环的执行频率。这行代码创建了一个名为 loop_rate 的 Rate 对象，频率设置为 80 Hz，这意味着程序的主循环将以每秒 80 次的频率运行。
    ros::Rate loop_rate(80);
    while (ros::ok()) {//ros::ok() 是一个ROS提供的函数，用于检查ROS节点是否还在运行。在这个循环中，通过检查 ros::ok() 的返回值来确定是否继续循环执行。
        process_detected_point(reference_point, detected_point, buffer);//根据视觉检测处理得到的点detected_point更新reference_point中的数据
        static airsim_ros_pkgs::VelCmd vel_cmd;
        vel_cmd = update_vel(reference_point, buffer, vel_cmd);//更新速度命令
        ROS_INFO("vel: x:%.2lf y:%.2lf z:%.2lf", vel_cmd.twist.linear.x, vel_cmd.twist.linear.y, vel_cmd.twist.linear.z);//打印速度命令信息

        // auto takeoff
        if (sqrt(pow(real_pose.pose.pose.position.x, 2)
        + pow(real_pose.pose.pose.position.y, 2)
        + pow(real_pose.pose.pose.position.z, 2)) <0.2) {
            vel_cmd.twist.linear.z = -1;
            vel_cmd.twist.linear.x = 0;
            vel_cmd.twist.linear.y = 0;
        }
        vel_publisher.publish(vel_cmd);//发布速度命令

        geometry_msgs::PointStamped real_position;
        real_position.point = real_pose.pose.pose.position;
        real_position.header.frame_id = "map";
        real_position.header.stamp = ros::Time::now();
        map_position_publisher.publish(real_position);
        ros::spinOnce();//处理回调函数和其他ROS消息
        loop_rate.sleep();//根据预设的循环速率 loop_rate 进行休眠，以控制循环的执行频率
    }
}