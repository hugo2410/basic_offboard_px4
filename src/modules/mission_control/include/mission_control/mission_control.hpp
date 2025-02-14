#ifndef MISSION_CONTROL_HPP
#define MISSION_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <chrono>
#include <string>

class MissionControl : public rclcpp::Node {
public:
    MissionControl();
    void postConstructionSetup();

private:
    // State variables
    mavros_msgs::msg::State current_state_;
    std::string mission_state_;
    bool armed_;
    double cruise_altitude_;
    double external_cmd_timeout_; // seconds
    geometry_msgs::msg::TwistStamped::SharedPtr external_velocity_command_;
    geometry_msgs::msg::Pose current_position_;
    rclcpp::Time last_external_cmd_time_;

    // ROS interfaces
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_setpoint_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback functions
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
    void positionCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void velocitySetpointCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    // Utility functions
    bool checkExternalCmdTimeout();
    void setMode(const std::string &mode);
    void arm(bool value);
    void initializeMission();
    void timerCallback();
};

#endif // MISSION_CONTROL_HPP
