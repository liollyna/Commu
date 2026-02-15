#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "frontier_detector.h"

class FrontierExplorer : public rclcpp::Node
{
public:
  FrontierExplorer()
  : Node("frontier_explorer")
  {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      rclcpp::QoS(10),
      std::bind(&FrontierExplorer::mapCallback, this, std::placeholders::_1));

    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/frontier_goal",
      10);

    RCLCPP_INFO(this->get_logger(), "Frontier explorer started");
  }

private:
  // =============================
  // CALLBACK MAP
  // =============================
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    int width  = msg->info.width;
    int height = msg->info.height;

    std::vector<int8_t> map = msg->data;

    std::vector<GridCell> frontiers;
    detectFrontiers(map, width, height, frontiers);

    if (frontiers.empty()) {
      RCLCPP_WARN(this->get_logger(), "No frontier found");
      return;
    }

    // 🎯 stratégie minimale : prendre la première frontière
    GridCell target = frontiers.front();

    publishGoal(target, msg);
  }

  // =============================
  // PUBLICATION GOAL
  // =============================
  void publishGoal(
    const GridCell& cell,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map_msg)
  {
    geometry_msgs::msg::PoseStamped goal;

    goal.header.stamp = this->now();
    goal.header.frame_id = "map";

    // conversion grille → monde
    float resolution = map_msg->info.resolution;
    float origin_x   = map_msg->info.origin.position.x;
    float origin_y   = map_msg->info.origin.position.y;

    goal.pose.position.x = origin_x + (cell.x + 0.5f) * resolution;
    goal.pose.position.y = origin_y + (cell.y + 0.5f) * resolution;
    goal.pose.position.z = 0.0;

    goal.pose.orientation.w = 1.0;

    goal_pub_->publish(goal);

    RCLCPP_INFO(this->get_logger(),
                "Frontier goal published: (%.2f, %.2f)",
                goal.pose.position.x,
                goal.pose.position.y);
  }

  // =============================
  // MEMBRES
  // =============================
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
};

// =============================
// MAIN
// =============================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierExplorer>());
  rclcpp::shutdown();
  return 0;
}
