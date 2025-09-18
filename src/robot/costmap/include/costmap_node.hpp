#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>

class CostmapNode : public rclcpp::Node {
public:
    CostmapNode();

private:
    // Callbacks
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void inflateObstacles();
    void publishCostmap();

    // ROS interfaces
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::TimerBase::SharedPtr timer_; 

    // Costmap parameters
    int width_;
    int height_;
    float resolution_;
    float inflation_radius_;
    std::vector<std::vector<int>> costmap2D_;
};

#endif // COSTMAP_NODE_HPP_