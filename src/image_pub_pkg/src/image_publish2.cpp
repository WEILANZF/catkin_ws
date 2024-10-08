#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("usb_cam/image_raw", 10);

    ROS_INFO("Opening camera...");
    VideoCapture cap(0);

    // 设置摄像头参数
    const int FRAME_WIDTH = 640;  // 示例宽度
    const int FRAME_HEIGHT = 480; // 示例高度
    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); // MJPEG 编码

    // 检查摄像头是否成功打开
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera.");
        return -1;
    }

    // 设置摄像头参数
    cap.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    cap.set(cv::CAP_PROP_FOURCC, fourcc);

    ROS_INFO("Camera opened!");

    Mat srcframe;
    ros::Rate loop_rate(10);

    while (nh.ok()) {
        cap >> srcframe; // 读取一帧
        if(srcframe.empty()) {
            ROS_WARN_ONCE("Frame is empty. Check camera connection.");
            continue;
        }

        // 注意：此处假设摄像头输出为彩色图像"bgr8"，且您想以彩色图像发布。如果是黑白图像，则应更改为"mono8",
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcframe).toImageMsg(); 

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO_ONCE("Sending images...");
    }

    cap.release(); // 释放摄像头资源
    return 0;
}