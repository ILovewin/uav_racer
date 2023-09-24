# uav_racer
The drone went through the loop at top speed

## 环境配置

采用LINUX + WIN 双机开发模式，WIN 端运行模拟器能提供最佳运行性能，双机模式分离后免除了ros 端程序对模拟器的影响，使得开发更便利。具体请参考RoboMaster官方[2022年自主无人机竞速模拟器使用说明](https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator)配置环境。

## 快速使用

**必须先启动模拟器后再开启roswrapper,否则会出现topic无数据的问题；关闭时要先退出roswrapper在关闭模拟器，否则模拟器会卡死**

1. ### WIN端启动模拟器	

   - 打开命令提示符，输入`ipconfig`命令，并按下回车键查看网络配置信息，选择合适的适配器的IPv4地址

   * 打开模拟器文件目录下的launcher.exe文件，输入上一步得到的IPv4地址，并按下回车键

   - 输入“0”选择模式，再按下回车键

2. ### LINUX端启动roswrapper

   - 进入roswrapper目录

     ```
     cd /path/to/IntelligentUAVChampionshipSimulator/roswrapper
     ```

   - 使用脚本启动roswrapper，参数为你的模拟器ip地址（前面启动模拟器输入的ip地址）

     ```
     ./simulator.sh 127.0.0.1
     ```

3. ### ROS软件包的构建、设置和运行过程

   - 在uav_racer目录下构建ROS软件包

     ```
     catkin_make
     ```

     若出现这样的报错信息**Invoking "make -j8 -l8" failed**，只需再执行一次**catkin_make**

   - 设置ROS环境变量，使得在当前终端中可以正确运行和使用ROS软件包（默认的shell不同，**devel** 目录中生成的脚本文件不同，所以设置ROS环境时，请确保与自己的默认shell相对应）

     ```
     source devel/setup.bash
     ```

   - 启动launch文件

     ```
     roslaunch planner run.launch
     ```

## 主要模块

|     软件包     |                             功能                             |
| :------------: | :----------------------------------------------------------: |
|   /src/basic   | 订阅无人机图像和位置信息，进行图像处理和深度坐标转换，并发布检测到的路径点消息 |
| /src/keybd_ctl |       实现通过键盘控制模拟器中无人机的起飞、降落和运动       |
|  /src/planner  | 订阅来自视觉处理的点信息，将这些点与参考路径点进行处理和匹配，然后根据匹配结果通过PID控制生成速度命令,从而控制无人机的运动 |

## 算法解析

1. ### 世界坐标系与相机坐标系间的转换

   使用[tf2_ros](http://wiki.ros.org/tf2_ros) 库发布、监听和应用坐标系之间的变换。它通过 `tf2_ros::TransformBroadcaster` 类发布变换，使用 `tf2_ros::TransformListener` 类监听变换，并使用 `tf2_ros::Buffer` 类应用变换

   - 发布变换

     创建一个名为`transform_puber`的`tf2_ros::TransformBroadcaster` 对象，用于发布坐标系之间的变换关系，并在 `real_pose_cb` 回调函数中，将数据头部的坐标系设置为 “map”，表示数据中的位置和姿态信息是相对于 “map” 坐标系来描述的，而子坐标系被设置为 “cam3d”，表示变换是从 “map” 坐标系到 “cam3d” 坐标系的。然后，这一回调函数还将无人机的位置信息和姿态信息被存入 `data.transform`中（**data**是**geometry_msgs::TransformStamped**类型的变量，用于描述两个坐标系之间的关系）。最后，在回调函数外部，使用 `transform_puber.sendTransform(data)` 发布变换

   - 监听变换

     创建一个`tf2_ros::Buffer` 对象`buffer`，用于存储和管理坐标变换的缓冲区。然后，创建一个名为`listener`的`tf2_ros::TransformListener` 对象，并将之前创建的 `buffer` 作为参数传递给它。这个监听器会订阅并接收 tf2 坐标变换的消息，并将其存储在 `buffer` 中

   - 应用变换

     使用 `tf2_ros::Buffer` 类中的`transform`方法来应用坐标系之间的变换。这个过程包括两个步骤：首先，应用平移变换，将点沿着 x、y、z 轴分别移动 `data.transform.translation.x`、`data.transform.translation.y` 和 `data.transform.translation.z` 的距离；然后，应用旋转变换，根据四元数 `data.transform.rotation` 来旋转点

2. ### 相机逆投影

   ```
   //通过相机内参，将圆环的二维深度图像坐标(像素坐标)转换为相机坐标系下的三维深度坐标
   this->center_point.point.x = point_pixel.point.z;
   this->center_point.point.y = (point_pixel.point.x - caminfo.K.at(2)) / caminfo.K.at(0) * point_pixel.point.z;
   this->center_point.point.z = (point_pixel.point.y - caminfo.K.at(5)) / caminfo.K.at(4) * point_pixel.point.z;
   ```

   逆投影： 逆投影是将深度图像中的像素坐标 `(u, v)` 和深度值 `z` 转换为相机坐标系下的三维深度坐标 `(X, Y, Z)` 的过程，所以在给定相机参数`caminfo.K`（**caminfo**是**geometry_msgs::PointStamped**的变量，在程序中通过订阅名为**airsim_node/drone_1/front_center/Scene/camera_info**的话题，再调用对应回调函数将相机信息传给这一变量）和深度图像中的像素坐标 `point_pixel` 的情况下，可以将圆环的二维深度图像坐标(像素坐标)转换为相机坐标系下的三维深度坐标

   逆投影公式为：

   ```
   X = (u - cx) * z / fx
   Y = (v - cy) * z / fy
   Z = z
   ```

   相机的内参是由相机内参矩阵 `K`获得，`K` 是一个9个元素的一维数组，为 `[fx 0 cx 0 fy cy 0 0 1]`，其中 fx 和 fy 是相机在 x 和 y 方向上的焦距在像素尺度，cx 和 cy 是主点在图像坐标系中的位置

   [参考资料](https://blog.csdn.net/kids_budong_c/article/details/125901134)

## 可优化模块

目前代码中的速度控制使用了比例控制项（P项）来计算线性速度的各分量，缺少了积分项和微分项，通过添加积分项和微分项可以改进控制器的性能，使其更稳定、更精确地控制无人机运动，并且可以针对不同轴使用不同的PID参数，调整PID参数来更好地满足系统需求

下面是尝试改写的代码：

```
struct PIDController{
    float Kp;//比例项
    float Ki;//积分项
    float Kd;//微分项
    float error_sum;//用于累积误差的变量，即积分项的值
    float last_error;//上一次的误差值，用于计算微分项的值

    PIDController(float Kp, float Ki, float Kd) : Kp(Kp), Ki(Ki), Kd(Kd), error_sum(0.0f), last_error(0.0f) {}
    
    float calculate(float error, float dt) {
        error_sum += error * dt;
        float error_rate = (error - last_error) / dt;
        last_error = error;
        return Kp * error + Ki * error_sum + Kd * error_rate;
    }

};

static PIDController pid_x(3.3f, 0.0f, 0.1f);
static PIDController pid_y(4.4f, 0.05f, 0.0f);
static PIDController pid_z(4.7f, 0.2f, 0.4f);

airsim_ros_pkgs::VelCmd control_pid(float x, float y, float z, int num) {//x,y,z为目标点的三轴方向的坐标，num表示第几个点

    airsim_ros_pkgs::VelCmd vel_cmd;
    
    // 计算 dt
    static auto last_time = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto dt = std::chrono::duration_cast<std::chrono::duration<float>>(now - last_time).count();
    last_time = now;
    
    // 使用 PID 控制器计算速度
    float vx = pid_x.calculate(x, dt);
    float vy = pid_y.calculate(y, dt);
    float vz = pid_z.calculate(z, dt);
    
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
    
    vel_cmd.twist.linear.x = vx;
    vel_cmd.twist.linear.y = vy;
    vel_cmd.twist.linear.z = vz;

#ifdef YAW_TRACK
    vel_cmd.twist.angular.z = Kp_YAW * atan(vy/vx);
#endif
    return vel_cmd;
}
```

- 经测试，速度控制改用这一代码穿圈所用时间与原代码穿圈所用时间基本一致，但稳定性不如原代码，仍需继续调整PID参数
- 下一步将针对特定路径调整PID参数和速度范围，原代码设置的最大速度为10.0，最小速度为8.0，改写这部分代码设置的最大速度为11.4，最小速度为8.4
- 无人机的起飞速度也可调整，原代码只设置了z轴（竖直）方向的线速度，改写的代码增加了x轴方向的线速度的设置，为1
- 改写的代码在无人机每穿越一个障碍环后需更新参数设置（可自定义一个set函数），避免持续累积误差

[比赛规则和技术链接](https://www.robomaster.com/zh-CN/resource/pages/announcement/1461)