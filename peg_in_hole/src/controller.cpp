
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <control_msgs/action/gripper_command.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  auto gripper_action_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
      move_group_node,
      "/panda_hand_controller/gripper_cmd");
  auto admittance_param_client = move_group_node->create_client<rcl_interfaces::srv::SetParameters>("/admittance_controller/set_parameters");

  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group.setPlannerId("PTP");

  RCLCPP_INFO(LOGGER, "Planning pipeline: %s", move_group.getPlanningPipelineId().c_str());
  RCLCPP_INFO(LOGGER, "Planner ID: %s", move_group.getPlannerId().c_str());
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

  rclcpp::Rate(0.25).sleep();

  if (!gripper_action_client->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(LOGGER, "Action server not available after waiting");
    rclcpp::shutdown();
  }

  if (!admittance_param_client->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(LOGGER, "Param server not available after waiting");
    rclcpp::shutdown();
  }

  auto gripper_goal_msg = control_msgs::action::GripperCommand::Goal();
  auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();

  gripper_goal_msg.command.position = 0.02;
  gripper_action_client->async_send_goal(gripper_goal_msg, send_goal_options);

  rclcpp::Rate(1).sleep();

  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 0;
  target_pose.orientation.x = -0.924;
  target_pose.orientation.y = 0.383;
  target_pose.orientation.z = 0;
  target_pose.position.x = 0.;
  target_pose.position.y = 0.5;
  target_pose.position.z = 0.35;
  move_group.setPoseTarget(target_pose);
  move_group.move();

  auto param_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  auto param = rcl_interfaces::msg::Parameter();
  param.name = "admittance.selected_axes";
  param.value.type = 6;
  param.value.bool_array_value = std::vector(6, false);
  param.value.bool_array_value.at(2) = true;
  param_request->parameters.push_back(param);
  admittance_param_client->async_send_request(param_request);

  rclcpp::Rate(1).sleep();

  target_pose.position.z = 0.08;
  move_group.setPoseTarget(target_pose);
  move_group.move();

  rclcpp::shutdown();
  return 0;
}