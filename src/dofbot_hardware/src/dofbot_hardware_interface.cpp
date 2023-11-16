#include <fmt/core.h>
#include <exception>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/macros.hpp>

#include "dofbot_hardware/dofbot_hardware_interface.hpp"

namespace dofbot_hardware{

    using StateInterface = hardware_interface::StateInterface;
    using CommandInterface = hardware_interface::CommandInterface;

    DofbotHardwareInterface::DofbotHardwareInterface(std::shared_ptr<Robot> robot)
        :DofbotHardwareInterface(){
            robot_ = std::move(robot);
    }

    DofbotHardwareInterface::DofbotHardwareInterface()
        :command_interfaces_info_({
            {hardware_interface::HW_IF_POSITION, kNumberofJoints, position_joint_interface_claimed_},
            {hardware_interface::HW_IF_VELOCITY, kNumberofJoints, velocity_joint_interface_claimed_},
            {k_HW_IF_CARTESIAN_VELOCITY, hw_cartesian_velocities_.size(),
            velocity_cartesian_interface_claimed_}
        }){}

    std::vector<StateInterface> DofbotHardwareInterface::export_state_interfaces(){
        std::vector<StateInterface> state_interfaces;
        for(int i = 0; i < info_.joints.size(); i++){
            state_interfaces.emplace_back(StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_.at(i)));
            state_interfaces.emplace_back(StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_.at(i)));
        }
        return state_interfaces;
    }

    std::vector<CommandInterface> DofbotHardwareInterface::export_command_interfaces(){
        std::vector<CommandInterface> command_interfaces;
        command_interfaces.reserve(info_.joints.size());
        for(int i = 0; i < info_.joints.size(); i++){
            command_interfaces.emplace_back(CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_.at(i)));
            command_interfaces.emplace_back(CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_.at(i)));
        }

        for(int i = 0; i < hw_cartesian_velocities_.size(); i++){
            command_interfaces.emplace_back(CommandInterface(hw_cartesian_velocities_names_.at(i),
                                                        k_HW_IF_CARTESIAN_VELOCITY,
                                                        &hw_cartesian_velocities_.at(i)));
        }

        return command_interfaces;
    }

    CallbackReturn DofbotHardwareInterface::on_activate(
        const rclcpp_lifecycle::State&){
            read(rclcpp::Time(0), rclcpp::Duration(0, 0));
            RCLCPP_INFO(getLogger(), "Started!");
            return CallbackReturn::SUCCESS;
        }
    
    CallbackReturn DofbotHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State&){
            RCLCPP_INFO(getLogger(), "trying to stop...");
            robot_->stopRobot();
            RCLCPP_INFO(getLogger(), "Stopped");
            return CallbackReturn::SUCCESS;
        }
    
    template <typename CommandType>
    void initializeCommand(bool& first_update,
                            const bool& interface_running,
                            CommandType& hw_command,
                            const CommandType& new_command){
        if(first_update && interface_running){
            hw_command = new_command;
            first_update = false;
        }
    } 

    void DofbotHardwareInterface::initializePositionCommands(const dofbot::RobotState& robot_state){
        initializeCommand(first_position_update_, position_joint_interface_runnning_,
                    hw_position_commands_, robot_state.q_d);
    }

    hardware_interface::return_type DofbotHardwareInterface::read(const rclcpp::Time&,
                                                                    const rclcpp::Duration&){
        
    }

    template <typename CommandType>
    bool hasInfinite(const CommandType& commands){
        return std::any_of(commands.begin(), commands.end(),
                            [](double command){return !std::isfinite(command);});
    }

    hardware_interface::return_type DofbotHardwareInterface::write(const rclcpp::Time&,
                                                                    const rclcpp::Duration&){

    }
}