#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
 
using namespace cv;
using namespace std;
 
int main(int argc,char** argv)
{
        ros::init(argc,argv,"image_publisher");
        ros::NodeHandle nh;
 
        image_transport::ImageTransport it(nh);
        image_transport::Publisher pub = it.advertise("cam0/image_raw",1);
 
        ROS_INFO("camera open ...");
        VideoCapture cam(0);
 
        if(!cam.isOpened())
        {
                exit(0);
        }
        ROS_INFO("camera opened!");
 
        Mat srcframe;
        ros::Rate loop_rate(10);
 
        while(nh.ok())
        {
                cam >> srcframe;
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"mono8",srcframe).toImageMsg();
                pub.publish(msg);
                ros::spinOnce();
                loop_rate.sleep();
                ROS_INFO("send image");
        }
}