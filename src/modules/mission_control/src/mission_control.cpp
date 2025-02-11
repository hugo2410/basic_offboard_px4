#include "include/mission_control.hpp"

using namespace std::chrono_literals;

MissionControl::MissionControl()
    : Node("mission_control_node"),
      armed_(false),
      current_state_("INIT"),
      cruise_altitude_(50.0),
      external_velocity_command_(nullptr),
      external_cmd_timeout_(0.5) {

    RCLCPP_INFO(this->get_logger(), "Initializing node");

    // Subscribers
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10, 
        std::bind(&MissionControl::stateCallback, this, std::placeholders::_1));

    position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/mavros/local_position/odom", 10, 
        std::bind(&MissionControl::positionCallback, this, std::placeholders::_1));

    velocity_setpoint_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/autonomy/velocity_setpoint", 10,
        std::bind(&MissionControl::velocitySetpointCallback, this, std::placeholders::_1));

    // Publisher
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10);

    // Service clients
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

    // Timer
    timer_ = this->create_wall_timer(
        300ms, std::bind(&MissionControl::timerCallback, this));

    // Wait for services
    while (!set_mode_client_->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for mode setting service...");
    }
    while (!arm_client_->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for arming service...");
    }

    RCLCPP_INFO(this->get_logger(), "Finished initializing node");
    initializeMission();
}

void MissionControl::stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state_ = *msg;
}

void MissionControl::positionCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_ = msg->pose.pose;
}

void MissionControl::velocitySetpointCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    external_velocity_command_ = msg;
    last_external_cmd_time_ = this->get_clock()->now();
}

bool MissionControl::checkExternalCmdTimeout() {
    if (external_velocity_command_ && 
        (this->get_clock()->now() - last_external_cmd_time_).seconds() > external_cmd_timeout_) {
        external_velocity_command_ = nullptr;
        return true;
    }
    return false;
}

void MissionControl::setMode(const std::string &