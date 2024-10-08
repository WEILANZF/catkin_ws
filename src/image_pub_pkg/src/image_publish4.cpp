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

        // 压缩图像以减少数据量
        //vector<uchar> buf;
        //imencode(".jpg", srcframe, buf); // 使用JPEG压缩
        //Mat compressed_image = imdecode(buf, IMREAD_COLOR); // 解压缩回到Mat格式
        //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", compressed_image).toImageMsg();


        vector<uchar> buf;
        imencode(".jpg", grayframe, buf); // 使用JPEG压缩
        Mat compressed_image = imdecode(buf, IMREAD_GRAYSCALE); // 解压缩回到Mat格式
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8" , compressed_image).toImageMsg();

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO_ONCE("Sending images...");
    }

    cap.release();
    return 0;
}
