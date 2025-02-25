// Copyright 2025 Hugo Birch
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "mission_control/mission_control.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionControl>();
  node->postConstructionSetup();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
