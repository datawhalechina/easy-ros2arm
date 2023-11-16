#ifndef _DOFBOT_HARDWARE_INTERFACE_HPP
#define _DOFBOT_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/visibility_control.h>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "dofbot_hardware/robot.hpp"
#include "dofbot_hardware/dofbot_param_service_server.hpp"
#include "dofbot_hardware/dofbot_executor.hpp"

#include "libdofbot/robot_state.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace dofbot_hardware{

class DofbotHardwareInterface : public hardware_interface::SystemInterface{
public:
    explicit DofbotHardwareInterface(std::shared_ptr<Robot> robot);
    DofbotHardwareInterface();
    DofbotHardwareInterface(const DofbotHardwareInterface&) = delete;
    DofbotHardwareInterface(DofbotHardwareInterface&& other) = delete;
    DofbotHardwareInterface& operator=(const DofbotHardwareInterface& other) = delete;
    DofbotHardwareInterface& operator=(const DofbotHardwareInterface&& other) = delete;
    ~DofbotHardwareInterface() override = default;

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;


    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time,
                                            const rclcpp::Duration& period) override;
    
    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    static const size_t kNumberofJoints = 6;

private:
    struct InterfaceInfo{
        std::string interface_type;
        size_t size;
        bool& claim_flag;
    };

    void initializePositionCommands(const dofbot::RobotState& robot_state);

    bool first_position_update_{true};

    std::shared_ptr<Robot> robot_;
    std::shared_ptr<DofbotParamServiceServer> node_;
    std::shared_ptr<DofbotExecutor> executor_;

    std::array<double, kNumberofJoints> hw_position_commands_{0, 0, 0, 0, 0, 0};
    std::array<double, kNumberofJoints> hw_velocity_commands_{0, 0, 0, 0, 0, 0};


    std::array<double, kNumberofJoints> hw_positions_{0, 0, 0, 0, 0, 0};
    std::array<double, kNumberofJoints> hw_velocities_{0, 0, 0, 0, 0, 0};

    std::array<std::string, 6> hw_cartesian_velocities_names_{"vx", "vy", "vz", "wx", "wy", "wz"};
    std::array<double, 6> hw_cartesian_velocities_{0, 0, 0, 0, 0, 0};

    const std::string k_HW_IF_CARTESIAN_VELOCITY = "cartesian_velocity";

    const std::vector<InterfaceInfo> command_interfaces_info_;

    // dofbot::RobotState hw_dofbot_robot_state_;
    // dofbot::RobotState* hw_dofbot_robot_state_addr_ = &hw_dofbot_robot_state_;
    // Model* hw_dofbot_model_ptr_ = nullptr;

    bool position_joint_interface_claimed_ = false;
    bool position_joint_interface_runnning_ = false;

    bool velocity_joint_interface_claimed_ = false;
    bool velocity_joint_interface_running_ = false;

    bool velocity_cartesian_interface_claimed_ = false;
    bool velocity_cartesian_interface_running_ = false;

    static rclcpp::Logger getLogger();

    const std::string k_robot_name{"dofbot"};
    const std::string k_robot_state_interface_name{"robot_state"};
    const std::string k_robot_model_interface_name{"robot_model"};

};

}

#endif