#ifndef HARUTO_OMNI_DRIVE_HARDWARE_INTERFACE__OMNI_DRIVE_SYSTEM_INTERFACE_H__
#define HARUTO_OMNI_DRIVE_HARDWARE_INTERFACE__OMNI_DRIVE_SYSTEM_INTERFACE_H__

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include <rclcpp/macros.hpp>

#include <haruto_omni_drive_hardware_interface/visibility_control.h>


namespace haruto_omni_drive_hardware_interface
{
class OmniDriveSystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OmniDriveSystemInterface);

  HARUTO_OMNI_DRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  HARUTO_OMNI_DRIVE_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  HARUTO_OMNI_DRIVE_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  HARUTO_OMNI_DRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  HARUTO_OMNI_DRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  HARUTO_OMNI_DRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  HARUTO_OMNI_DRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  HARUTO_OMNI_DRIVE_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_accelerations_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_accelerations_;

  double hw_start_sec_;
  double hw_stop_sec_;

  enum integration_level_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
    ACCELERATION = 3
  };

  std::vector<integration_level_t> control_level_;
};
}

#endif // HARUTO_OMNI_DRIVE_HARDWARE_INTERFACE__OMNI_DRIVE_SYSTEM_INTERFACE_H__
