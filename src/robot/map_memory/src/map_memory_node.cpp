#include "map_memory_node.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

MapMemoryNode::MapMemoryNode()
    : Node("map_memory_node"),
      last_x(0.0),
      last_y(0.0),
      distance_threshold(0.0),//switch back to a positive number after testing
      costmap_updated_(false),
      should_update_map_(false),
      first_costmap_received_(false)
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
    if (global_map_.data.empty()) {
        // Initialize global map from first costmap
        global_map_ = latest_costmap_;
        return;
    }
    global_map_ = latest_costmap_;
    /* for (int i = 0; i < latest_costmap_.data.size(); ++i) {
        int val = latest_costmap_.data[i];
        if (val >= 0) {  // Known cell (occupied or free)
            global_map_.data[i] = val;
        }
    } */
    
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}