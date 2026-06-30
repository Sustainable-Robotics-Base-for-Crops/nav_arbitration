// Copyright 2026 SABI AGRI

#ifndef NAV_ARBITRATION__ARBITRATION_HPP_
#define NAV_ARBITRATION__ARBITRATION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "nav_interfaces/msg/conductor.hpp"
#include "simple_node/node.hpp"
#include "yasmin/state_machine.hpp"

namespace nav_arbitration
{
class Arbitration : public simple_node::Node
{
public:
  explicit Arbitration();
  void start_sm();

protected:
  void conductor_callback(const nav_interfaces::msg::Conductor::SharedPtr msg);
  void replay_status_callback(const std_msgs::msg::UInt64::SharedPtr msg);
  void vehicle_status_callback(const std_msgs::msg::UInt64::SharedPtr msg);
  void timer_callback();

private:
  std::vector<std::string> node_names_;
  std::string node_name_line_matcher_server_{ "/auto/line/matcher" };
  std::string node_name_line_follower_{ "/auto/line/follower" };
  std::string node_name_turn_on_spot_server_{ "/auto/turn/on_spot" };
  std::string node_name_path_matcher_server_{ "/auto/path/matcher" };
  std::string node_name_path_follower_{ "/auto/path/follower" };

  std::string node_name_replay_{ "/auto/replay" };
  std::string node_name_recorder_{ "/auto/recorder" };
  std::string node_name_geofencing_{ "/safety/geofencing/geofencing_publisher" };

  std::shared_ptr<yasmin::Blackboard> blackboard_;
  std::shared_ptr<yasmin::StateMachine> state_machine_;

  std::shared_ptr<rclcpp::Node> client_service_node_;

  nav_interfaces::msg::Conductor conductor_state_msg_;

  rclcpp::Publisher<nav_interfaces::msg::Conductor>::SharedPtr conductor_state_pub_;
  rclcpp::Subscription<nav_interfaces::msg::Conductor>::SharedPtr conductor_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr replay_status_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr vehicle_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace nav_arbitration

#endif  // NAV_ARBITRATION__ARBITRATION_HPP_
