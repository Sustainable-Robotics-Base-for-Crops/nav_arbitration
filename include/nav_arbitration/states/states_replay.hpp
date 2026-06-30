// Copyright 2026 SABI AGRI

#ifndef NAV_ARBITRATION__STATES__STATES_REPLAY_HPP_
#define NAV_ARBITRATION__STATES__STATES_REPLAY_HPP_

#include "nav_arbitration/states/states_arbitration.hpp"
#include "nav_arbitration/states/arbitration_outcomes.hpp"

namespace nav_arbitration
{
class ReplayInactiveState : public BaseState
{
public:
  ReplayInactiveState(std::shared_ptr<yasmin::Blackboard> blackboard)
    : BaseState({ outcomes::CLEANUP, outcomes::ACTIVATE }, blackboard)
  {
    current_state_lifecycle_.id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
  };
  ~ReplayInactiveState(){};

  bool is_state_transition(std::string& outcome) override
  {
    if (conductor_cmd_.restart || conductor_cmd_.drive_mode != nav_interfaces::msg::Conductor::DRIVE_REPLAY ||
        (vehicle_status_ >= (uint64_t(0x01) << 32) &&
         (vehicle_status_ & uint64_t(VehicleStatus::warn_emergency_stop_remote)) == 0 &&
         (vehicle_status_ & uint64_t(VehicleStatus::warn_bumper)) == 0 &&
         (clock_.now() - time_end_of_emergency_stop_).seconds() > timeout_end_emergency_stop_))
    {
      if (change_replay_states(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED))
      {
        if (change_action_servers_states(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED))
        {
          outcome = outcomes::CLEANUP;
          return true;
        }
        change_replay_states(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
      }
    }
    else if (!conductor_cmd_.restart && !conductor_cmd_.pause && replay_status_ < (uint64_t(0x01) << 32) &&
             vehicle_status_ < (uint64_t(0x01) << 32) &&
             (vehicle_status_ & uint64_t(VehicleStatus::warn_emergency_stop_remote)) == 0 &&
             (vehicle_status_ & uint64_t(VehicleStatus::warn_bumper)) == 0)
    {
      if (change_replay_states(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE))
      {
        if (change_action_servers_states(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE))
        {
          outcome = outcomes::ACTIVATE;
          return true;
        }
        change_replay_states(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
      }
    }
    conductor_cmd_.drive_mode = nav_interfaces::msg::Conductor::DRIVE_REPLAY;
    conductor_cmd_.pause = true;
    conductor_cmd_.restart = false;
    blackboard_->set<nav_interfaces::msg::Conductor>("conductor_cmd", conductor_cmd_);
    return false;
  }
  std::string to_string() const override
  {
    return states_names[States::REPLAY_INACTIVE];
  }
};

class ReplayActiveState : public BaseState
{
public:
  ReplayActiveState(std::shared_ptr<yasmin::Blackboard> blackboard)
    : BaseState({ outcomes::CLEANUP, outcomes::DEACTIVATE }, blackboard)
  {
    current_state_lifecycle_.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  };
  ~ReplayActiveState(){};

  bool is_state_transition(std::string& outcome) override
  {
    if (conductor_cmd_.restart || conductor_cmd_.drive_mode != nav_interfaces::msg::Conductor::DRIVE_REPLAY ||
        (vehicle_status_ >= (uint64_t(0x01) << 32) &&
         (vehicle_status_ & uint64_t(VehicleStatus::warn_emergency_stop_remote)) == 0 &&
         (vehicle_status_ & uint64_t(VehicleStatus::warn_bumper)) == 0) ||
        (replay_status_ & 0x01) == 0x01)  // put in unconfigured state if vehicle error or end of path
    {
      if (change_replay_states(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) &&
          change_action_servers_states(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED))
      {
        outcome = outcomes::CLEANUP;
        conductor_cmd_.drive_mode = nav_interfaces::msg::Conductor::DRIVE_TELEOP;
        conductor_cmd_.pause = true;
        conductor_cmd_.restart = true;
        blackboard_->set<nav_interfaces::msg::Conductor>("conductor_cmd", conductor_cmd_);
        return true;
      }
    }
    else if (!conductor_cmd_.restart && (conductor_cmd_.pause || replay_status_ >= (uint64_t(0x01) << 32) ||
                                         (vehicle_status_ & uint64_t(VehicleStatus::warn_emergency_stop_remote)) != 0 ||
                                         (vehicle_status_ & uint64_t(VehicleStatus::warn_bumper)) != 0))
    {
      if (change_replay_states(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) &&
          change_action_servers_states(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE))
      {
        outcome = outcomes::DEACTIVATE;
        return true;
      }
    }
    conductor_cmd_.drive_mode = nav_interfaces::msg::Conductor::DRIVE_REPLAY;
    conductor_cmd_.pause = false;
    conductor_cmd_.restart = false;
    blackboard_->set<nav_interfaces::msg::Conductor>("conductor_cmd", conductor_cmd_);
    return false;
  }
  std::string to_string() const override
  {
    return states_names[States::REPLAY_ACTIVE];
  }
};
}  // namespace nav_arbitration

#endif  // NAV_ARBITRATION__STATES__STATES_REPLAY_HPP_
