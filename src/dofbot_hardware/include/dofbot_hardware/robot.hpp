#ifndef _ROBOT_HPP
#define _ROBOT_HPP

#include <array>
#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/logger.hpp>

#include "libdofbot/robot_state.h"

namespace dofbot_hardware{

class Robot{
public:
    explicit Robot(const std::string& robot_ip, const rclcpp::Logger& logger);
    Robot(const Robot&) = delete;
    Robot& operator=(const Robot& other) = delete;
    Robot& operator=(Robot&& other) = delete;
    Robot(Robot&& other) = delete;

    virtual ~Robot();

    virtual void initializeJointPositionInterface();

    virtual void initializeJointVelocityInterface();

    virtual void initializeCartesianVelocityInterface();

    virtual void stopRobot();

    virtual dofbot::RobotState readOnce();
};

}

#endif