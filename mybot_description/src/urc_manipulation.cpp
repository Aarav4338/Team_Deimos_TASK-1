#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "urc_manipulation",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("urc_manipulation");

  // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Fetch parameters from move_group node (required when using ros2 run in Humble)
  RCLCPP_INFO(logger, "Fetching parameters from move_group...");
  auto param_node = std::make_shared<rclcpp::Node>("urc_manipulation_param_fetcher");
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(param_node, "move_group");
  while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) return 0;
    RCLCPP_INFO(logger, "Waiting for move_group parameter service...");
  }
  auto parameters = parameters_client->get_parameters({
    "robot_description",
    "robot_description_semantic",
    "robot_description_kinematics.arm.kinematics_solver",
    "robot_description_kinematics.arm.kinematics_solver_search_resolution",
    "robot_description_kinematics.arm.kinematics_solver_timeout"
  });
  for (const auto& param : parameters) {
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      node->declare_parameter(param.get_name(), param.as_string());
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      node->declare_parameter(param.get_name(), param.as_double());
    }
  }

  // Create the MoveIt MoveGroup Interface for the "arm" group
  using moveit::planning_interface::MoveGroupInterface;

  auto move_group_interface = MoveGroupInterface(node, "arm");

  // Create the Planning Scene Interface to add collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  // 2. Spawn the Collision Object in the middle
  RCLCPP_INFO(logger, "Spawning collision object in the middle...");
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();
  collision_object.id = "obstacle_cube";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.2;
  primitive.dimensions[primitive.BOX_Y] = 0.2;
  primitive.dimensions[primitive.BOX_Z] = 0.2; // A perfect cube

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.40;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.25;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface.applyCollisionObjects(collision_objects);
  rclcpp::sleep_for(std::chrono::seconds(1)); // Wait for scene to update


  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
