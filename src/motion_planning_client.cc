#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <chrono>
#include <string>
#include <boost/program_options.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "ros2_motion_planning/srv/motion_planning_service.hpp"

namespace po = boost::program_options;

void build_request_markers(
  const std::shared_ptr<rclcpp::Node>& node,
  const std::shared_ptr<ros2_motion_planning::srv::MotionPlanningService::Request>& request,
  visualization_msgs::msg::MarkerArray& marker_array
) {
  
  visualization_msgs::msg::Marker start_marker;
  start_marker.header.frame_id = "map";
  start_marker.header.stamp = node->get_clock()->now();
  start_marker.id = 0;
  start_marker.ns = "start_goal";
  start_marker.type = visualization_msgs::msg::Marker::SPHERE;
  start_marker.action = visualization_msgs::msg::Marker::ADD;
  start_marker.pose.position.x = request->start.x;
  start_marker.pose.position.y = request->start.y;
  start_marker.pose.position.z = 0.0;
  start_marker.pose.orientation.w = 1.0;
  start_marker.scale.x = 1.0;
  start_marker.scale.y = 1.0;
  start_marker.scale.z = 1.0;

  // Green for Start
  start_marker.color.r = 0.0;
  start_marker.color.g = 1.0;
  start_marker.color.b = 0.0; 
  start_marker.color.a = 1.0;

  visualization_msgs::msg::Marker goal_marker;
  goal_marker.header.frame_id = "map";
  goal_marker.header.stamp = node->get_clock()->now();
  goal_marker.id = 1;
  goal_marker.ns = "start_goal";
  goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  goal_marker.action = visualization_msgs::msg::Marker::ADD;
  goal_marker.pose.position.x = request->goal.x;
  goal_marker.pose.position.y = request->goal.y;
  goal_marker.pose.position.z = 0.0;
  goal_marker.pose.orientation.w = 1.0;
  goal_marker.scale.x = 1.0;
  goal_marker.scale.y = 1.0;
  goal_marker.scale.z = 1.0;

  // Red for Goal
  goal_marker.color.r = 1.0;
  goal_marker.color.g = 0.0;
  goal_marker.color.b = 0.0; 
  goal_marker.color.a = 1.0;

  start_marker.lifetime = rclcpp::Duration::from_nanoseconds(0);
  goal_marker.lifetime = rclcpp::Duration::from_nanoseconds(0);
  marker_array.markers.push_back(start_marker);
  marker_array.markers.push_back(goal_marker);

}

bool load_occupancy_grid_from_pgm(const std::string& filename, nav_msgs::msg::OccupancyGrid& grid_msg) {

  std::ifstream infile(filename);
  if (!infile.is_open()) return false;

  std::string magic_number;
  std::getline(infile, magic_number); // P2
  
  std::string line;
  while (infile.peek() == '#') { // Skip comments
    std::getline(infile, line); 
  }

  int width = 0;
  int height = 0;
  int max_val = 0;
  infile >> width >> height >> max_val;

  grid_msg.info.resolution = 0.2;
  grid_msg.info.width = width;
  grid_msg.info.height = height;
  grid_msg.info.origin.position.x = -128 * 0.2;
  grid_msg.info.origin.position.y = -128 * 0.2;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;

  grid_msg.data.resize(width * height);

  for (int row = 0; row < height; ++row) {

    for (int col = 0; col < width; ++col) {

      int index = width * (height - row - 1) + col;
      int pixel_val = 0;
      infile >> pixel_val;
      grid_msg.data[index] = static_cast<int8_t>(pixel_val - 128);

    }

  }

  return true;

}

int main(int argc, char* argv[]) {

  po::options_description desc{"Motion Planning Client Options"};
  desc.add_options()
    ("help,h", "Produce help message")
    ("map,m", po::value<std::string>(), "Path to PGM map file (required)")
    ("algorithm,a", po::value<std::string>()->default_value("astar"), "Planning algorithm ('astar' or 'rrt')")
    ("animate", po::value<bool>()->default_value(false), "Enable live search exploration visualization in RViz")
    ("start-x,x0", po::value<double>()->default_value(-20.0), "Start position X (m)")
    ("start-y,y0", po::value<double>()->default_value(0.0), "Start position Y (m)")
    ("start-theta,t0", po::value<double>()->default_value(0.0), "Start heading angle (rad)")
    ("goal-x,xf", po::value<double>()->default_value(20.0), "Goal position X (m)")
    ("goal-y,yf", po::value<double>()->default_value(0.0), "Goal position Y (m)")
    ("goal-theta,tf", po::value<double>()->default_value(0.0), "Goal heading angle (rad)");

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).allow_unregistered().run(), vm);
  po::notify(vm);

  if (vm.count("help")) {

    std::cout << desc << std::endl;
    return EXIT_SUCCESS;

  }

  if (!vm.count("map")) {

    std::cerr << "Error: Map file path must be specified via --map option." << std::endl;
    return EXIT_FAILURE;

  }

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("motion_planning_client");

  auto grid_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 1);
  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 1);
  auto path_pub = node->create_publisher<nav_msgs::msg::Path>("/plan", 10);

  auto client = node->create_client<ros2_motion_planning::srv::MotionPlanningService>("planning_query");

  auto request = std::make_shared<ros2_motion_planning::srv::MotionPlanningService::Request>();
  request->start.x = vm["start-x"].as<double>();
  request->start.y = vm["start-y"].as<double>();
  request->start.theta = vm["start-theta"].as<double>();

  request->goal.x = vm["goal-x"].as<double>();
  request->goal.y = vm["goal-y"].as<double>();
  request->goal.theta = vm["goal-theta"].as<double>();

  request->algorithm = vm["algorithm"].as<std::string>();
  request->animate = vm["animate"].as<bool>();

  request->map.header.frame_id = "map";
  request->map.header.stamp = node->get_clock()->now();

  std::string map_file = vm["map"].as<std::string>();
  RCLCPP_INFO(node->get_logger(), "Loading map from: %s", map_file.c_str());
  if (!load_occupancy_grid_from_pgm(map_file, request->map)) {

    RCLCPP_ERROR(node->get_logger(), "Failed to read map file: %s", map_file.c_str());
    return EXIT_FAILURE;

  }

  while (!client->wait_for_service(std::chrono::seconds(1))) {

    if (!rclcpp::ok()) {

      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for planner service.");
      return EXIT_FAILURE;

    }

    RCLCPP_INFO(node->get_logger(), "Waiting for 'planning_query' service...");

  }

  visualization_msgs::msg::MarkerArray marker_array;
  build_request_markers(node, request, marker_array);

  grid_pub->publish(request->map);
  marker_pub->publish(marker_array);

  RCLCPP_INFO(node->get_logger(), "Sending planning request using algorithm: %s", request->algorithm.c_str());
  auto result_future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {

    auto response = result_future.get();
    RCLCPP_INFO(node->get_logger(), "Planning successful! Received path with %zu poses.", response->plan.poses.size());
    path_pub->publish(response->plan);
    grid_pub->publish(request->map);
    marker_pub->publish(marker_array);

  } else {

    RCLCPP_ERROR(node->get_logger(), "Failed to receive response from planner service.");

  }

  RCLCPP_INFO(node->get_logger(), "Planning client finished request.");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return EXIT_SUCCESS;

}
