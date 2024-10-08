#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;
//减少重复压缩/解压缩：考虑直接发布灰度图而不进行压缩和解压缩，这样可以进一步减少计算开销。
//调整发布频率：根据实际需求调整发布频率，以平衡图像质量和性能。

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("usb_cam/image_raw", 10); // 增加发布队列大小

    ROS_INFO("Opening camera...");
    VideoCapture cap(0);

    const int FRAME_WIDTH = 640;
    const int FRAME_HEIGHT = 480;
    int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera.");
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    cap.set(cv::CAP_PROP_FOURCC, fourcc);

    ROS_INFO("Camera opened!");

    Mat srcframe, grayframe;
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        cap >> srcframe;
        if (srcframe.empty()) {
            ROS_WARN_ONCE("Frame is empty. Check camera connection.");
            continue;
        }

        cv::cvtColor(srcframe, grayframe, cv::COLOR_BGR2GRAY);

        // 直接发布灰度图
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", grayframe).toImageMsg();

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO_ONCE("Sending images...");
    }

    cap.release();
    return 0;
}
