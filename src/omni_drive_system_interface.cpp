#include <haruto_omni_drive_hardware_interface/omni_drive_system_interface.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace haruto_omni_drive_hardware_interface
{
  hardware_interface::CallbackReturn OmniDriveSystemInterface::on_init(
    const hardware_interface::HardwareInfo & info)
  {

  }

  std::vector<hardware_interface::StateInterface>
    OmniDriveSystemInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_states_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_states_velocities_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_ACCELERATION,
        &hw_states_accelerations_[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
    OmniDriveSystemInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_commands_positions_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_commands_velocities_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_ACCELERATION,
        &hw_commands_accelerations_[i]));
    }
    return command_interfaces;
  }

  hardware_interface::return_type OmniDriveSystemInterface::prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces)
  {
    std::vector<integration_level_t> new_modes = {};
    for (std::string key : start_interfaces)
    {
      for (std::size_t i = 0; i < info_.joints.size(); i++)
      {
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
        {
          new_modes.push_back(integration_level_t::POSITION);
        }
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
        {
          new_modes.push_back(integration_level_t::VELOCITY);
        }
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_ACCELERATION)
        {
          new_modes.push_back(integration_level_t::ACCELERATION);
        }
      }
    }

    if (new_modes.size() != info_.joints.size())
    {
      return hardware_interface::return_type::ERROR;
    }

    if (!std::all_of(
          new_modes.begin() + 1, new_modes.end(),
          [&](integration_level_t mode) { return mode == new_modes[0]; }))
    {
      return hardware_interface::return_type::ERROR;
    }

    for (std::string key : stop_interfaces)
    {
      for (std::size_t i = 0; i < info_.joints.size(); i++)
      {
        if (key.find(info_.joints[i].name) != std::string::npos)
        {
          hw_commands_velocities_[i] = 0;
          hw_commands_accelerations_[i] = 0;
          control_level_[i] = integration_level_t::UNDEFINED; 
        }
      }
    }

    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (control_level_[i] != integration_level_t::UNDEFINED)
      {
        return hardware_interface::return_type::ERROR;
      }
      control_level_[i] = new_modes[i];
    }
    return hardware_interface::return_type::OK;    
  }

  hardware_interface::CallbackReturn OmniDriveSystemInterface::on_activate(
    const rclcpp_lifecycle::State & previous_state)
  {
    for (int i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    for (std::size_t i = 0; i < hw_states_positions_.size(); i++)
    {
      if (std::isnan(hw_states_positions_[i]))
      {
        hw_states_positions_[i] = 0;
      }
      if (std::isnan(hw_states_velocities_[i]))
      {
        hw_states_velocities_[i] = 0;
      }
      if (std::isnan(hw_states_accelerations_[i]))
      {
        hw_states_accelerations_[i] = 0;
      }
      if (std::isnan(hw_commands_positions_[i]))
      {
        hw_commands_positions_[i] = 0;
      }
      if (std::isnan(hw_commands_velocities_[i]))
      {
        hw_commands_velocities_[i] = 0;
      }
      if (std::isnan(hw_commands_accelerations_[i]))
      {
        hw_commands_accelerations_[i] = 0;
      }
      control_level_[i] = integration_level_t::UNDEFINED;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn OmniDriveSystemInterface::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
  {
    for (int i = 0; i < hw_stop_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }

  hardware_interface::return_type OmniDriveSystemInterface::read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
  {

  }

  hardware_interface::return_type OmniDriveSystemInterface::write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
  {

  }
}  // namespace haruto_omni_drive_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  haruto_omni_drive_hardware_interface::OmniDriveSystemInterface,
  hardware_interface::SystemInterface)
