#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
using namespace hardware_interface;

namespace quadruped_hardware_sim
{
class CANMotorSim : public SystemInterface
{
  std::vector<double> cmd_pos_, cmd_vel_, cmd_eff_;
  std::vector<double> state_pos_, state_vel_, state_eff_;
  std::vector<std::string> joint_names_;

public:
  CallbackReturn on_init(const HardwareInfo & info) override
  {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
      return CallbackReturn::ERROR;

    for (auto & joint : info.joints)
      joint_names_.push_back(joint.name);

    size_t n = info.joints.size();
    cmd_pos_.resize(n, 0.0);
    cmd_vel_.resize(n, 0.0);
    cmd_eff_.resize(n, 0.0);
    state_pos_.resize(n, 0.0);
    state_vel_.resize(n, 0.0);
    state_eff_.resize(n, 0.0);

    RCLCPP_INFO(rclcpp::get_logger("CANMotorSim"), "Initialized simulated hardware for %zu joints", n);
    return CallbackReturn::SUCCESS;
  }

  std::vector<StateInterface> export_state_interfaces() override
  {
    std::vector<StateInterface> state_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      state_interfaces.emplace_back(joint_names_[i], HW_IF_POSITION, &state_pos_[i]);
      state_interfaces.emplace_back(joint_names_[i], HW_IF_VELOCITY, &state_vel_[i]);
      state_interfaces.emplace_back(joint_names_[i], HW_IF_EFFORT, &state_eff_[i]);
    }
    return state_interfaces;
  }

  std::vector<CommandInterface> export_command_interfaces() override
  {
    std::vector<CommandInterface> command_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      command_interfaces.emplace_back(joint_names_[i], HW_IF_POSITION, &cmd_pos_[i]);
      command_interfaces.emplace_back(joint_names_[i], HW_IF_VELOCITY, &cmd_vel_[i]);
      command_interfaces.emplace_back(joint_names_[i], HW_IF_EFFORT, &cmd_eff_[i]);
    }
    return command_interfaces;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // For sim, mirror commands into state
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      state_pos_[i] = cmd_pos_[i];
      state_vel_[i] = cmd_vel_[i];
      state_eff_[i] = cmd_eff_[i];
    }
    return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // Simulate motor response by applying commanded positions to the states
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      // Move gradually toward the commanded position (simple physics approximation)
      double error = cmd_pos_[i] - state_pos_[i];
      state_vel_[i] = error * 10.0;   // proportional velocity
      state_pos_[i] += state_vel_[i] * 0.001;  // integrate small time step
    }
    return return_type::OK;
  }

};
}  // namespace quadruped_hardware_sim

PLUGINLIB_EXPORT_CLASS(quadruped_hardware_sim::CANMotorSim, hardware_interface::SystemInterface)
