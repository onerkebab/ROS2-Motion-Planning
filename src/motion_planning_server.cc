#include <iostream>
#include <memory>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ros2_motion_planning/srv/motion_planning_service.hpp"
#include "ros2_motion_planning/astar_planner.h"
#include "ros2_motion_planning/rrt_planner.h"

void handle_planning_request(
  const std::shared_ptr<ros2_motion_planning::srv::MotionPlanningService::Request>& request,
  const std::shared_ptr<ros2_motion_planning::srv::MotionPlanningService::Response>& response,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& vis_pub
) {
  
  RCLCPP_INFO(rclcpp::get_logger("motion_planning_server"), "Received path planning request.");
  RCLCPP_INFO(
    rclcpp::get_logger("motion_planning_server"), "Start: (%.2f, %.2f, %.2f) -> Goal: (%.2f, %.2f, %.2f)",
    request->start.x, 
    request->start.y, 
    request->start.theta,
    request->goal.x, 
    request->goal.y, 
    request->goal.theta
  );

  std::string algo = request->algorithm;
  std::transform(algo.begin(), algo.end(), algo.begin(), ::tolower);

  bool success = false;
  auto active_pub = request->animate ? vis_pub : nullptr;

  if (algo == "astar" || algo == "a*") {

    RCLCPP_INFO(rclcpp::get_logger("motion_planning_server"), "Executing A* Search (deterministic) Planner...");
    motion_planning::AStarPlanner planner;
    success = planner.generate_path(request, response, active_pub);

  } else if (algo == "rrt") {
    
    RCLCPP_INFO(rclcpp::get_logger("motion_planning_server"), "Executing RRT (probabilistic) Planner...");
    motion_planning::RRTPlanner planner;
    success = planner.generate_path(request, response, active_pub);

  } else {

    RCLCPP_ERROR(
      rclcpp::get_logger("motion_planning_server"), "Unsupported algorithm: '%s'. Supported algorithms are 'astar' and 'rrt'.", request->algorithm.c_str());
    return;

  }

  if (success) {

    response->plan.header.frame_id = "map";
    response->plan.header.stamp = rclcpp::Clock().now();
    RCLCPP_INFO(rclcpp::get_logger("motion_planning_server"), "Path planning succeeded! Path contains %zu poses.", response->plan.poses.size());

  } else {

    RCLCPP_ERROR(rclcpp::get_logger("motion_planning_server"), "Path planning failed to find a valid route.");

  }

}

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("motion_planning_server");
  auto vis_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("search_visualization", 10);

  auto service = node->create_service<ros2_motion_planning::srv::MotionPlanningService>(
    "planning_query",
    [vis_pub](const std::shared_ptr<ros2_motion_planning::srv::MotionPlanningService::Request>& request,
              const std::shared_ptr<ros2_motion_planning::srv::MotionPlanningService::Response>& response) {
      handle_planning_request(request, response, vis_pub);
    });

  RCLCPP_INFO(rclcpp::get_logger("motion_planning_server"), "Motion planning server is ready.");
  
  rclcpp::spin(node);
  rclcpp::shutdown();

  return EXIT_SUCCESS;

}
