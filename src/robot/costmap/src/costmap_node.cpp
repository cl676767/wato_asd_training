#include "costmap_node.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

CostmapNode::CostmapNode()
    : Node("costmap"),
      width_(100),
      height_(100),
      resolution_(0.1),
      inflation_radius_(1),
      costmap2D_(height_, std::vector<int>(width_, 0))
{
    // Publisher
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&CostmapNode::publishCostmap, this)
    );

    // Subscriber
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 10,
        std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1)
    );
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Reset costmap
    for (int y = 0; y < height_; y++)
        for (int x = 0; x < width_; x++)
            costmap2D_[y][x] = 0;

    // Convert laser scan points to grid
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        if (range < scan->range_max && range > scan->range_min) {
            int x_grid = static_cast<int>(range * std::cos(angle) / resolution_ + width_ / 2);
            int y_grid = static_cast<int>(range * std::sin(angle) / resolution_ + height_ / 2);

            if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_)
                costmap2D_[y_grid][x_grid] = 100;
        }
    }

    inflateObstacles();
    publishCostmap();
}

void CostmapNode::inflateObstacles() {
    int cellRadius = static_cast<int>(inflation_radius_ / resolution_);

    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {
            if (costmap2D_[y][x] != 100) continue;

            for (int dy = -cellRadius; dy <= cellRadius; dy++) {
                for (int dx = -cellRadius; dx <= cellRadius; dx++) {
                    int nx = x + dx;
                    int ny = y + dy;

                    if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

                    double dist = std::hypot(dx * resolution_, dy * resolution_);
                    if (dist <= inflation_radius_ && dist > 0) {
                        int cost = static_cast<int>(100 * (1 - dist / inflation_radius_));
                        costmap2D_[ny][nx] = std::max(costmap2D_[ny][nx], cost);
                    }
                }
            }
        }
    }
}

void CostmapNode::publishCostmap() {
    nav_msgs::msg::OccupancyGrid message;
    message.info.width = width_;
    message.info.height = height_;
    message.info.resolution = resolution_;
    message.info.origin.position.x = width_ / 2.0 * resolution_;
    message.info.origin.position.y = height_ / 2.0 * resolution_;
    message.info.origin.position.z = 0.0;
    message.info.origin.orientation.w = 1.0; // identity quaternion

    message.header.stamp = this->now();
    message.header.frame_id = "map";

    message.data.resize(width_ * height_);
    for (int y = 0; y < height_; y++)
        for (int x = 0; x < width_; x++)
            message.data[y * width_ + x] = costmap2D_[y][x];

    costmap_pub_->publish(message);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}