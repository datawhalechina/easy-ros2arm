#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "dofbot_hardware/robot.hpp"

namespace dofbot_hardware{

class DofbotParamServiceServer : public rclcpp::Node{
public:
    DofbotParamServiceServer(const rclcpp::NodeOptions& options, std::shared_ptr<Robot> robot);
};

}
