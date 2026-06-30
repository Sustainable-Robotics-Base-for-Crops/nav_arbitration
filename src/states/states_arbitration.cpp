// Copyright 2026 SABI AGRI

#include "nav_arbitration/states/states_arbitration.hpp"
#include "nav_arbitration/states/arbitration_outcomes.hpp"
#include "nav_lifecycle_manager/lifecycle_manager.hpp"

namespace nav_arbitration
{
// _______________________________BASE_STATE__________________________________________
BaseState::BaseState(std::set<std::string> outcomes, std::shared_ptr<yasmin::Blackboard> blackboard)
  : yasmin::State(outcomes)
{
  blackboard_ = blackboard;
  timeout_end_emergency_stop_ = blackboard_->get<float>("timeout_end_emergency_stop");
  current_state_lifecycle_.id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
}

bool BaseState::change_action_servers_states(lifecycle_msgs::msg::State::_id_type desired_state)
{
  bool result = blackboard_->get<std::shared_ptr<nav_lifecycle_manager::LifecycleManager>>("lifecycle_manager")
                    ->change_state_all(desired_state);
  return result;
}

bool BaseState::change_replay_states(lifecycle_msgs::msg::State::_id_type desired_state)
{
  bool result = blackboard_->get<std::shared_ptr<nav_lifecycle_manager::LifecycleServiceClient>>("client_replay")
                    ->change_state_robust(desired_state);
  result &= blackboard_->get<std::shared_ptr<nav_lifecycle_manager::LifecycleServiceClient>>("client_geofencing")
                ->change_state_robust(desired_state);
  result &= blackboard_->get<std::shared_ptr<nav_lifecycle_manager::LifecycleServiceClient>>("client_json_agri_format")
                ->change_state_robust(desired_state);
  print_change_result(result, desired_state);
  return result;
}

bool BaseState::change_all_states(lifecycle_msgs::msg::State::_id_type desired_state)
{
  bool result = blackboard_->get<std::shared_ptr<nav_lifecycle_manager::LifecycleManager>>("lifecycle_manager")
                    ->change_state_all(desired_state);
  if (result == true)
  {
    result &= blackboard_->get<std::shared_ptr<nav_lifecycle_manager::LifecycleServiceClient>>("client_replay")
                  ->change_state_robust(desired_state);
  }

  print_change_result(result, desired_state);
  return result;
}

std::string BaseState::execute(std::shared_ptr<yasmin::Blackboard> /* blackboard */)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("ArbitrationStates"), "Executing " << to_string());
  std::string outcome{ outcomes::CLEANUP };
  while (rclcpp::ok())
  {
    control_loop_.sleep();
    try
    {
      update_blackboard();
      if (is_state_transition(outcome))
      {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("ArbitrationStates"), "execute transition outcome " << outcome);
        if (outcome == outcomes::CLEANUP)
        {
          blackboard_->get<std::shared_ptr<nav_lifecycle_manager::LifecycleServiceClient>>("client_recorder")
              ->change_state_robust(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
        }
        return outcome;
      }
    }
    catch (std::string& s)
    {
      RCLCPP_WARN_STREAM_THROTTLE(rclcpp::get_logger("ArbitrationStates"), clock_, 1000, s.c_str());
    }
  }
  if (to_string() == states_names[States::UNCONFIGURED])
  {
    outcome = outcomes::SHUTDOWN;
  }
  return outcome;
}

bool BaseState::is_state_transition(std::string& /* outcome */)
{
  return true;
}

void BaseState::print_change_result(bool result, uint8_t lc_state)
{
  std::string lc_state_string{ "Unknown" };
  if (lc_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    lc_state_string = "Active";
  }
  else if (lc_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    lc_state_string = "Inactive";
  }
  else if (lc_state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    lc_state_string = "Unconfigured";
  }
  if (result == true)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("ArbitrationStates"), "SUCCESS go to " << lc_state_string);
  }
  else
  {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("ArbitrationStates"), "FAILED  go to " << lc_state_string);
  }
}

void BaseState::update_blackboard()
{
  conductor_cmd_ = blackboard_->get<nav_interfaces::msg::Conductor>("conductor_cmd");
  replay_status_ = blackboard_->get<uint64_t>("replay_status");
  uint64_t vehicle_status = blackboard_->get<uint64_t>("vehicle_status");

  if (((vehicle_status_ & uint64_t(VehicleStatus::warn_emergency_stop_remote)) != 0 &&
       (vehicle_status & uint64_t(VehicleStatus::warn_emergency_stop_remote)) == 0) ||
      ((vehicle_status_ & uint64_t(VehicleStatus::warn_bumper)) != 0 &&
       (vehicle_status & uint64_t(VehicleStatus::warn_bumper)) == 0))
  {
    time_end_of_emergency_stop_ = clock_.now();
  }

  vehicle_status_ = vehicle_status;
}
}  // namespace nav_arbitration
