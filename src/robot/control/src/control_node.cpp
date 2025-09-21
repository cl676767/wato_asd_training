#include "control_node.hpp"
#include <cmath>

ControlNode::ControlNode() : Node("control_node") {
    // Parameters
    lookahead_distance_ = 1.0;  // meters
    goal_tolerance_ = 0.1;      // meters
    linear_speed_ = 0.5;        // m/s

    // Subscribers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) {
            current_path_ = msg;
        });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            robot_odom_ = msg;
        });

    // Publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer for control loop (10 Hz)
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
    if (!current_path_ || !robot_odom_) return;

    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        stopRobot();  // Stop if no valid point or at goal
        return;
    }

    auto cmd_vel = computeVelocity(*lookahead_point);
    cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    const auto& poses = current_path_->poses;
    if (poses.empty()) return std::nullopt;

    geometry_msgs::msg::Point robot_pos = robot_odom_->pose.pose.position;

    for (const auto& pose : poses) {
        double dist = computeDistance(robot_pos, pose.pose.position);
        if (dist >= lookahead_distance_) {
            return pose;
        }
    }

    // End of path
    double dist_to_goal = computeDistance(robot_pos, poses.back().pose.position);
    if (dist_to_goal <= goal_tolerance_) {
        return std::nullopt;  // Goal reached
    }
    return poses.back();  // Last point as fallback
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
    geometry_msgs::msg::Twist cmd_vel;

    geometry_msgs::msg::Point robot_pos = robot_odom_->pose.pose.position;
    double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

    double dx = target.pose.position.x - robot_pos.x;
    double dy = target.pose.position.y - robot_pos.y;

    double angle_to_target = std::atan2(dy, dx);
    double alpha = angle_to_target - robot_yaw;

    // Normalize alpha to [-pi, pi]
    while (alpha > M_PI) alpha -= 2 * M_PI;
    while (alpha < -M_PI) alpha += 2 * M_PI;

    double Ld = computeDistance(robot_pos, target.pose.position);  // Lookahead distance
    double k = 2.0 * std::sin(alpha) / Ld;

    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = k * linear_speed_;

    return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

void ControlNode::stopRobot() {
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}