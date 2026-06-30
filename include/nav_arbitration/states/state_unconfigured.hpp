// Copyright 2026 SABI AGRI

#ifndef NAV_ARBITRATION__STATES__STATE_UNCONFIGURED_HPP_
#define NAV_ARBITRATION__STATES__STATE_UNCONFIGURED_HPP_

#include "nav_arbitration/states/states_arbitration.hpp"
#include "nav_arbitration/states/arbitration_outcomes.hpp"
#include "nav_lifecycle_manager/lifecycle_service_client.hpp"

namespace nav_arbitration
{
class UnconfiguredState : public BaseState
{
public:
  UnconfiguredState(std::shared_ptr<yasmin::Blackboard> blackboard)
    : BaseState({ outcomes::CLEANUP, outcomes::REPLAY_CONFIGURE, outcomes::SHUTDOWN }, blackboard)
  {
    current_state_lifecycle_.id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
  };
  ~UnconfiguredState(){};

  bool is_state_transition(std::string& outcome) override
  {
    blackboard_->get<std::shared_ptr<nav_lifecycle_manager::LifecycleServiceClient>>("client_recorder")
        ->change_state_robust(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    if (change_action_servers_states(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) == false)
    {
      RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger("ArbitrationStates"), clock_, 1000,
                                  "Action servers can not leave UnconfiguredState");
    }
    else if (!conductor_cmd_.restart && (vehicle_status_ < (uint64_t(0x01) << 32) ||
                                         (vehicle_status_ & uint64_t(VehicleStatus::warn_emergency_stop_remote)) != 0 ||
                                         (vehicle_status_ & uint64_t(VehicleStatus::warn_bumper)) != 0))
    {
      if (conductor_cmd_.drive_mode == nav_interfaces::msg::Conductor::DRIVE_REPLAY &&
          change_replay_states(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE))
      {
        outcome = outcomes::REPLAY_CONFIGURE;
        return true;
      }
    }
    else
    {
      RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger("ArbitrationStates"), clock_, 1000,
                                  "Can not leave UnconfiguredState ; status vehicle : "
                                      << std::hex << vehicle_status_ << " replay : " << std::hex << replay_status_);
    }
    conductor_cmd_.drive_mode = nav_interfaces::msg::Conductor::DRIVE_TELEOP;
    conductor_cmd_.pause = true;
    conductor_cmd_.restart = true;
    blackboard_->set<nav_interfaces::msg::Conductor>("conductor_cmd", conductor_cmd_);
    return false;
  }
  std::string to_string() const override
  {
    return states_names[States::UNCONFIGURED];
  }
};
}  // namespace nav_arbitration

#endif  // NAV_ARBITRATION__STATES__STATE_UNCONFIGURED_HPP_
