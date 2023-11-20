#include <chrono>
#include <memory>
#include <string>
#include <functional>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"

#include "dofbot_msgs/msg/imageinfo.hpp"

using namespace std::chrono_literals;


class PicturePublisher : public rclcpp::Node
{
public:
    PicturePublisher() : Node("picture_publisher")
    {
        publisher_ = this->create_publisher<dofbot_msgs::msg::Imageinfo>("Camera_Images", 5);
        
        timer_ = this->create_wall_timer(
            500ms, std::bind(&PicturePublisher::timer_callback, this));
    }
private:
    void timer_callback()
    {
        auto imgInfo = dofbot_msgs::msg::Imageinfo();
        cv::Mat frame;
        cv::VideoCapture cap(10);
        if(!cap.isOpened()){
            RCLCPP_ERROR(this->get_logger(), "Unable to open camera");
            return;
        }
        cap >> frame;
        if(frame.empty()){
            RCLCPP_ERROR(this->get_logger(), "blank frame grabbed");
            return;
        }
        imgInfo.height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        imgInfo.width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        imgInfo.channel = cap.get(cv::CAP_PROP_CHANNEL);
        imgInfo.image = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg());
        publisher_->publish(imgInfo);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<dofbot_msgs::msg::Imageinfo>::SharedPtr publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PicturePublisher>());
    rclcpp::shutdown();
    return 0;
}