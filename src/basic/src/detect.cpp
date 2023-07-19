//
// Created by airvoltex on 22-8-23.
//
#include "basic/basic.hpp"



//main api
void VisionDetector::detect() {
    //设置中心点的坐标系为 "cam3d"
    this->center_point.header.frame_id = "cam3d";
    this->center_point.header.stamp = ros::Time::now();

    try {
        //获取圆环中心的像素坐标
        auto point_pixel = get_ring_orthogonal();

        // tested
        //通过相机内参，将圆环的二维深度图像坐标(像素坐标)转换为相机坐标系下的三维深度坐标
        this->center_point.point.x = point_pixel.point.z;
        this->center_point.point.y = (point_pixel.point.x - caminfo.K.at(2)) / caminfo.K.at(0) * point_pixel.point.z;
        this->center_point.point.z = (point_pixel.point.y - caminfo.K.at(5)) / caminfo.K.at(4) * point_pixel.point.z;

    }
    catch (cv::Exception &e) {
        ROS_WARN("%s\n", e.what());
    }

}

// input mask, return x and y for pixels and z for depth
geometry_msgs::PointStamped VisionDetector::get_ring_orthogonal() {

    geometry_msgs::PointStamped result;
    result.header.frame_id = "pixel";
    result.header.stamp = ros::Time::now();
    result.point.z = 0.0;
    try {
        int height = this->mask.rows;
        int width = this->mask.cols;
        float centerX = 0;
        float centerY = 0;
        int pixCount = 0;
        float centerDepth = 0;
        //遍历图像遮罩，找到所有有效像素点的位置和深度
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                if (this->mask.at<uchar>(i, j) > 0) {//找到符合颜色范围的像素点,并将像素点的列坐标和行坐标累加
                    centerX += (float) j;
                    centerY += (float) i;
                    pixCount++;
                    centerDepth += depth_by_color_image_scale(i, j);//获取当前像素点的深度值，该值通过深度图像缩放得到
                }
            }
        }

        if (pixCount < 30) {//如果有效像素点数目过少，则将中心点设为图像中心，并将深度设为0
            centerDepth = 0;
            centerY = height/2;
            centerX = width/2;
            this->detected_or_not = not_detected;
        }
        else
        {//计算有效像素点的中心位置和平均深度
            centerX /= (float) pixCount;
            centerY /= (float) pixCount;
            centerDepth /= (float) pixCount;
            if(centerDepth < 1e-3) {
                //根据原图得到的深度图在10m之外是无法识别的，所以如果深度小于1e-3（即无效深度），将深度设为10
                centerDepth = 10;
            }
            this->detected_or_not = detected;
        }
        //将中心点的位置和深度值存储在result.point变量中
        result.point.x = centerX;
        result.point.y = centerY;
        result.point.z = centerDepth;
    }
    catch (cv::Exception& e) {
        ROS_WARN("%s\n", e.what());
    }


    return result;
}

// 深度图的大小比原本彩色图小两倍，所以中心位置也要比原图小两倍,这个函数将原图中心点的坐标除以2，以获取对应的深度值
float VisionDetector::depth_by_color_image_scale(int i, int j) {
    return this->src_depth.at<float>(i/2, j/2);
}