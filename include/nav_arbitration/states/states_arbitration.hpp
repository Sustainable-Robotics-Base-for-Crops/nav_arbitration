// Copyright 2026 SABI AGRI

#ifndef NAV_ARBITRATION__STATES__STATES_ARBITRATION_HPP_
#define NAV_ARBITRATION__STATES__STATES_ARBITRATION_HPP_

#include "rclcpp/rate.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "yasmin/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_interfaces/msg/conductor.hpp"

#include <map>

namespace nav_arbitration
{
enum class States
{
  UNCONFIGURED,
  REPLAY_INACTIVE,
  REPLAY_ACTIVE
};

enum class VehicleStatus : uint64_t
{
  // Info status [0-15] bits
  normal_operation = 0x00,

  // Warn status [16-31] bits
  warn_emergency_stop = uint64_t(0x01) << 16,
  warn_emergency_stop_remote = uint64_t(0x02) << 16,
  warn_bumper = uint64_t(0x04) << 16,

  // Error status [32-63] bits
  error_udp_driver = uint64_t(0x01) << 32,
  error_battery = uint64_t(0x02) << 32,
  error_cylinder = uint64_t(0x04) << 32,
  error_vcu = uint64_t(0x08) << 32,
  error_motor = uint64_t(0x10) << 32,
  error_imu = uint64_t(0x20) << 32
};

static std::map<States, std::string> states_names = { { States::UNCONFIGURED, "unconfigured" },
                                                      { States::REPLAY_INACTIVE, "replay_inactive" },
                                                      { States::REPLAY_ACTIVE, "replay_active" } };

class BaseState : public yasmin::State
{
public:
  BaseState(std::set<std::string> outcomes, std::shared_ptr<yasmin::Blackboard> blackboard);
  ~BaseState(){};

  bool change_action_servers_states(lifecycle_msgs::msg::State::_id_type desired_state);
  bool change_replay_states(lifecycle_msgs::msg::State::_id_type desired_state);
  bool change_all_states(lifecycle_msgs::msg::State::_id_type desired_state);
  std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard);
  virtual bool is_state_transition(std::string& outcome);
  void print_change_result(bool result, uint8_t lc_state);
  void update_blackboard();

  std::shared_ptr<yasmin::Blackboard> blackboard_;
  rclcpp::WallRate control_loop_{ 10. };
  rclcpp::Clock clock_;
  lifecycle_msgs::msg::State current_state_lifecycle_;
  nav_interfaces::msg::Conductor conductor_cmd_;
  uint64_t replay_status_{ 0 };
  uint64_t vehicle_status_{ 0 };
  float timeout_end_emergency_stop_{ 8. };
  rclcpp::Time time_end_of_emergency_stop_{ 0, 0, RCL_SYSTEM_TIME };
};
}  // namespace nav_arbitration

#endif  // NAV_ARBITRATION__STATES__STATES_ARBITRATION_HPP_
