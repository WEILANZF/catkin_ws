#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

using namespace cv;
using namespace std;

const int MAX_QUEUE_SIZE = 10;

class ImageSender
{
public:
    ImageSender(ros::NodeHandle& nh);
    ~ImageSender();
    void startPublishing();
    void stopPublishing();

private:
    void sendImages();
    void publishImages();

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_; // 使用image_transport
    VideoCapture cap_; 
    std::thread sender_thread_, publisher_thread_; 
    std::queue<sensor_msgs::Image> image_queue_; 
    std::mutex queue_mutex_; 
    std::condition_variable queue_cv_; 
    bool stop_ = false; 
};

ImageSender::ImageSender(ros::NodeHandle& nh) : nh_(nh),
    it_(nh),
    image_pub_(it_.advertise("/usb_cam/image_raw", 1)), // 发布sensor_msgs::Image话题
    cap_(0) 
{
    if (!cap_.isOpened())
    {
        ROS_ERROR("Failed to open camera.");
        exit(-1);
    }
}

ImageSender::~ImageSender()
{
    stopPublishing();
}

void ImageSender::sendImages()
{
    Mat frame;
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    while (ros::ok() && !stop_)
    {
        cap_ >> frame;
        if(frame.empty())
        {
            ROS_WARN("Blank frame grabbed.");
            continue;
        }
        
        img_bridge = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame);
        img_bridge.toImageMsg(img_msg);

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if(image_queue_.size() < MAX_QUEUE_SIZE)
                image_queue_.push(img_msg);
            else
                ROS_WARN_ONCE("Queue is full, dropping frames.");
        }
        queue_cv_.notify_one();
    }
}

void ImageSender::publishImages()
{
    sensor_msgs::Image img_msg;
    while (ros::ok())
    {
        {
            unique_lock<mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this]{return !image_queue_.empty() || stop_;});
            if(stop_ && image_queue_.empty()) break;
            img_msg = image_queue_.front();
            image_queue_.pop();
        }
        image_pub_.publish(img_msg); // 发布sensor_msgs::Image消息
    }
}

void ImageSender::startPublishing()
{
    sender_thread_ = std::thread(&ImageSender::sendImages, this);
    publisher_thread_ = std::thread(&ImageSender::publishImages, this);
}

void ImageSender::stopPublishing()
{
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        stop_ = true;
    }
    queue_cv_.notify_all();
    if(sender_thread_.joinable()) sender_thread_.join();
    if(publisher_thread_.joinable()) publisher_thread_.join();
    cap_.release();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    ImageSender imageSender(nh);
    imageSender.startPublishing();
    ros::spin();
    imageSender.stopPublishing();
    return 0;
}
