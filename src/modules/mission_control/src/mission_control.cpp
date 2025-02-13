#include "mission_control/mission_control.hpp"

using namespace std::chrono_literals;

MissionControl::MissionControl()
    : Node("mission_control_node"),
      mission_state_("INIT"), // Initialize mission state as "INIT"
      armed_(false),
      cruise_altitude_(50.0),
      external_cmd_timeout_(0.5),
      external_velocity_command_(nullptr) {

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
    while (rclcpp::ok() && !set_mode_client_->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for mode setting service...");
    }
    while (rclcpp::ok() && !arm_client_->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Waiting for arming service...");
    }

    if (!rclcpp::ok()) {
        return;
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

void MissionControl::setMode(const std::string &mode) {
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;

    auto future = set_mode_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) == 
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->mode_sent) {
            RCLCPP_INFO(this->get_logger(), "Mode changed to %s successfully.", mode.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set mode to %s.", mode.c_str());
        }
    }
}

void MissionControl::arm(bool value) {
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = value;

    auto future = arm_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) == 
        rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            armed_ = value;
            RCLCPP_INFO(this->get_logger(), "Arming successful.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Arming failed.");
        }
    }
}

void MissionControl::initializeMission() {
    setMode("OFFBOARD");
    rclcpp::sleep_for(4s);
    arm(true);
}

void MissionControl::timerCallback() {
    if (!armed_) {
        mission_state_ = "INIT";
        return;
    }

    auto vel_cmd = geometry_msgs::msg::TwistStamped();
    vel_cmd.header.stamp = this->get_clock()->now();
    vel_cmd.header.frame_id = "base_link";

    if (mission_state_ == "INIT") {
        if (current_position_.position.z < cruise_altitude_) {
            vel_cmd.twist.linear.z = 2.0;
        } else {
            mission_state_ = "CRUISE";
            RCLCPP_INFO(this->get_logger(), "Reached cruise altitude, transitioning to cruise");
        }
    } else if (mission_state_ == "CRUISE") {
        if (!checkExternalCmdTimeout() && external_velocity_command_) {
            vel_cmd.twist = external_velocity_command_->twist;
        } else {
            vel_cmd.twist.linear.x = 0.0;
            vel_cmd.twist.linear.y = 0.0;
            vel_cmd.twist.linear.z = 0.0;
        }
    }

    vel_pub_->publish(vel_cmd);
}
