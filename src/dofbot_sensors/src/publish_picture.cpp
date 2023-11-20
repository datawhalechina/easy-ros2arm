#include <chrono>
#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "dofbot_msgs/msg/images.hpp"

using namespace std::chrono_literals;


class PicturePublisher : public rclcpp::Node
{
public:
    PicturePublisher() : Node("picture_publisher")
    {

    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PicturePublisher>());
    rclcpp::shutdown();
    return 0;
}