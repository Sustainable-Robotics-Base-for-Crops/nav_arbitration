// Copyright 2026 SABI AGRI

#include "nav_arbitration/arbitration.hpp"
#include "nav_arbitration/states/states_replay.hpp"
#include "nav_arbitration/states/state_unconfigured.hpp"
#include "nav_lifecycle_manager/lifecycle_manager.hpp"

namespace nav_arbitration
{
Arbitration::Arbitration()
  : simple_node::Node("arbitration", rclcpp::NodeOptions(),
                      std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
{
  float rate{ 0.3 };
  this->declare_parameter("rate", rate);
  this->get_parameter("rate", rate);
  float timeout_end_emergency_stop{ 8. };
  this->declare_parameter("timeout_end_emergency_stop", timeout_end_emergency_stop);
  this->get_parameter("timeout_end_emergency_stop", timeout_end_emergency_stop);
  this->declare_parameter("lateral_deviation_max", 0.4);
  this->declare_parameter("lateral_deviation_max.in_working_zone", 0.4);
  this->declare_parameter("lateral_deviation_max.out_working_zone", 0.6);
  this->declare_parameter("lateral_deviation_max.uturn", 1.5);
  this->declare_parameter("course_deviation_max", M_PI / 8);
  this->declare_parameter("course_deviation_max.uturn", M_PI / 3);
  this->declare_parameter("speed_working_zone_added", 0.0);
  this->declare_parameter("working_zone_action.name", "cylinder_go_end");
  this->declare_parameter("working_zone_action.offset_distance_at_the_start", 0.0);
  this->declare_parameter("working_zone_action.offset_distance_at_the_end", 0.0);
  this->declare_parameter("loop_back.return_speed", 0.0);

  // Initialize blackboard
  blackboard_ = std::make_shared<yasmin::Blackboard>();
  blackboard_->set<float>("timeout_end_emergency_stop", timeout_end_emergency_stop);
  client_service_node_ = this->create_sub_node("client_service");

  node_names_ = { node_name_line_matcher_server_, node_name_line_follower_, node_name_turn_on_spot_server_,
                  node_name_path_matcher_server_, node_name_path_follower_, node_name_cylinder_go_end_ };

  blackboard_->set<std::shared_ptr<nav_lifecycle_manager::LifecycleManager>>(
      "lifecycle_manager",
      std::make_shared<nav_lifecycle_manager::LifecycleManager>(client_service_node_, node_names_));

  blackboard_->set<std::shared_ptr<nav_lifecycle_manager::LifecycleServiceClient>>(
      "client_recorder",
      std::make_shared<nav_lifecycle_manager::LifecycleServiceClient>(node_name_recorder_, client_service_node_));
  blackboard_->set<std::shared_ptr<nav_lifecycle_manager::LifecycleServiceClient>>(
      "client_geofencing",
      std::make_shared<nav_lifecycle_manager::LifecycleServiceClient>(node_name_geofencing_, client_service_node_));
  blackboard_->set<std::shared_ptr<nav_lifecycle_manager::LifecycleServiceClient>>(
      "client_json_agri_format_parser", std::make_shared<nav_lifecycle_manager::LifecycleServiceClient>(
                                            node_name_json_agri_format_parser_, client_service_node_));
  blackboard_->set<std::shared_ptr<nav_lifecycle_manager::LifecycleServiceClient>>(
      "client_replay",
      std::make_shared<nav_lifecycle_manager::LifecycleServiceClient>(node_name_replay_, client_service_node_));

  blackboard_->set<nav_interfaces::msg::Conductor>("conductor_cmd", nav_interfaces::msg::Conductor{});
  blackboard_->set<uint64_t>("replay_status", 0);
  blackboard_->set<uint64_t>("vehicle_status", 0);

  state_machine_ = std::make_shared<yasmin::StateMachine>(std::set<std::string>{ outcomes::SHUTDOWN });
  state_machine_->add_state(states_names[States::UNCONFIGURED], std::make_shared<UnconfiguredState>(blackboard_),
                            { { outcomes::REPLAY_CONFIGURE, states_names[States::REPLAY_INACTIVE] },
                              { outcomes::CLEANUP, states_names[States::UNCONFIGURED] } });
  state_machine_->add_state(states_names[States::REPLAY_INACTIVE], std::make_shared<ReplayInactiveState>(blackboard_),
                            { { outcomes::CLEANUP, states_names[States::UNCONFIGURED] },
                              { outcomes::ACTIVATE, states_names[States::REPLAY_ACTIVE] } });
  state_machine_->add_state(states_names[States::REPLAY_ACTIVE], std::make_shared<ReplayActiveState>(blackboard_),
                            { { outcomes::CLEANUP, states_names[States::UNCONFIGURED] },
                              { outcomes::DEACTIVATE, states_names[States::REPLAY_INACTIVE] } });
  state_machine_->set_start_state(states_names[States::UNCONFIGURED]);

  std::chrono::duration<float> d(rate);
  conductor_state_pub_ = create_publisher<nav_interfaces::msg::Conductor>("/auto/conductor_state", 10);
  conductor_sub_ = create_subscription<nav_interfaces::msg::Conductor>(
      "/auto/conductor_cmd", rclcpp::QoS(1).best_effort(), std::bind(&Arbitration::conductor_callback, this, _1));
  replay_status_sub_ = create_subscription<std_msgs::msg::UInt64>(
      "replay/status", 10, std::bind(&Arbitration::replay_status_callback, this, _1));
  vehicle_status_sub_ = create_subscription<std_msgs::msg::UInt64>(
      "/vehicle/status", 10, std::bind(&Arbitration::vehicle_status_callback, this, _1));
  timer_ = create_wall_timer(d, std::bind(&Arbitration::timer_callback, this));
}

void Arbitration::start_sm()
{
  try
  {
    std::string outcome = (*state_machine_.get())(blackboard_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Outcome : " << outcome);
  }
  catch (std::string& s)
  {
    RCLCPP_WARN_STREAM(get_logger(), "catch state machine error" << s.c_str());
  }
}

void Arbitration::conductor_callback(const nav_interfaces::msg::Conductor::SharedPtr msg)
{
  blackboard_->set<nav_interfaces::msg::Conductor>("conductor_cmd", *msg);
}

void Arbitration::replay_status_callback(const std_msgs::msg::UInt64::SharedPtr msg)
{
  blackboard_->set<uint64_t>("replay_status", msg->data);
}

void Arbitration::vehicle_status_callback(const std_msgs::msg::UInt64::SharedPtr msg)
{
  blackboard_->set<uint64_t>("vehicle_status", msg->data);
}

void Arbitration::timer_callback()
{
  std::string current_state = state_machine_->get_current_state();
  RCLCPP_DEBUG_STREAM(this->get_logger(), "current state " << current_state);

  if (current_state == states_names[States::UNCONFIGURED])
  {
    conductor_state_msg_.drive_mode = nav_interfaces::msg::Conductor::DRIVE_TELEOP;
    conductor_state_msg_.pause = true;
    conductor_state_msg_.restart = true;
  }
  else if (current_state == states_names[States::REPLAY_INACTIVE])
  {
    conductor_state_msg_.drive_mode = nav_interfaces::msg::Conductor::DRIVE_REPLAY;
    conductor_state_msg_.pause = true;
    conductor_state_msg_.restart = false;
  }
  else if (current_state == states_names[States::REPLAY_ACTIVE])
  {
    conductor_state_msg_.drive_mode = nav_interfaces::msg::Conductor::DRIVE_REPLAY;
    conductor_state_msg_.pause = false;
    conductor_state_msg_.restart = false;
  }

  conductor_state_pub_->publish(conductor_state_msg_);
}
}  // namespace nav_arbitration

int main(int argc, char* argv[])
{
  try
  {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav_arbitration::Arbitration>();
    node->start_sm();
    node->join_spin();
    rclcpp::shutdown();
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }

  return 0;
}
