# uav_racer
The drone went through the loop at top speed

## 环境配置

采用LINUX + WIN 双机开发模式，WIN 端运行模拟器能提供最佳运行性能，双机模式分离后免除了ros 端程序对模拟器的影响，使得开发更便利。具体请参考RoboMaster官方[自主无人机竞速模拟器使用说明](https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator)配置环境。

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
- 改写的代码在无人机每穿越一个障碍环后需调用PIDController（）函数，更新参数设置，避免持续累积误差

[比赛规则和技术链接](https://www.robomaster.com/zh-CN/resource/pages/announcement/1461)