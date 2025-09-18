#include "map_memory_node.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

MapMemoryNode::MapMemoryNode()
    : Node("map_memory_node"),
      last_x(0.0),
      last_y(0.0),
      distance_threshold(5.0),
      costmap_updated_(false),
      should_update_map_(false)
{
    // Subscribers
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10,
        std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10,
        std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
    );

    // Publisher
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MapMemoryNode::updateMap, this)
    );

    // Initialize global map
    global_map_.info.resolution = 0.1;
    global_map_.info.width = 300;
    global_map_.info.height = 300;
    global_map_.info.origin.position.x = 0.0;
    global_map_.info.origin.position.y = 0.0;
    global_map_.info.origin.orientation.w = 1.0; // identity quaternion
    global_map_.data.resize(global_map_.info.width * global_map_.info.height, 0);
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_costmap_ = *msg;
    costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    double distance = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
    if (distance >= distance_threshold) {
        last_x = x;
        last_y = y;
        should_update_map_ = true;
    }
}

void MapMemoryNode::updateMap() {
    if (should_update_map_ && costmap_updated_) {
        integrateCostmap();
        map_pub_->publish(global_map_);
        should_update_map_ = false;
    }
}

void MapMemoryNode::integrateCostmap() {
    for (int y = 0; y < int(latest_costmap_.info.height); y++) {
        for (int x = 0; x < int(latest_costmap_.info.width); x++) {
            int local_index = y * latest_costmap_.info.width + x;
            int value = latest_costmap_.data[local_index];
            if (value < 0) continue; // skip unknown

            int gx = static_cast<int>(
                (latest_costmap_.info.origin.position.x + x * latest_costmap_.info.resolution - last_x) 
                / global_map_.info.resolution
            );
            int gy = static_cast<int>(
                (latest_costmap_.info.origin.position.y + y * latest_costmap_.info.resolution - last_y) 
                / global_map_.info.resolution
            );

            if (gx < 0 || gy < 0 || gx >= int(global_map_.info.width) || gy >= int(global_map_.info.height)) continue;

            int global_index = gy * global_map_.info.width + gx;
            global_map_.data[global_index] = std::max(int(global_map_.data[global_index]), value);
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}