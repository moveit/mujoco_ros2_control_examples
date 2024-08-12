
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit_msgs/srv/get_motion_sequence.hpp>
#include <moveit/kinematic_constraints/utils.h>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  auto gripper_action_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
      move_group_node,
      "/panda_hand_controller/gripper_cmd");
  auto admittance_param_client = move_group_node->create_client<rcl_interfaces::srv::SetParameters>("/admittance_controller/set_parameters");

  auto service_client = move_group_node->create_client<moveit_msgs::srv::GetMotionSequence>("/plan_sequence_path");

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

  if (!service_client->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(LOGGER, "/plan_sequence_path server not available after waiting");
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
  target_pose.position.x = 0.5;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.35;
  move_group.setPoseTarget(target_pose);
  move_group.move();

  auto param_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  auto param = rcl_interfaces::msg::Parameter();
  param.name = "admittance.selected_axes";
  param.value.type = 6;
  param.value.bool_array_value = std::vector(6, false);
  param.value.bool_array_value.at(0) = true;
  param.value.bool_array_value.at(1) = true;
  param.value.bool_array_value.at(2) = true;
  param_request->parameters.push_back(param);
  admittance_param_client->async_send_request(param_request);

  rclcpp::Rate(1).sleep();

  moveit_msgs::msg::MotionSequenceItem item1;
  item1.blend_radius = 0.0;

  item1.req.group_name = PLANNING_GROUP;
  item1.req.planner_id = "PTP";
  item1.req.allowed_planning_time = 5.0;
  item1.req.max_velocity_scaling_factor = 0.05;
  item1.req.max_acceleration_scaling_factor = 0.05;

  moveit_msgs::msg::Constraints constraints_item1;
  moveit_msgs::msg::PositionConstraint pos_constraint_item1;
  pos_constraint_item1.header.frame_id = "panda_link0";
  pos_constraint_item1.link_name = "panda_hand";

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "panda_link0";
  pose_stamped.pose = target_pose;
  pose_stamped.pose.position.z = 0.2;

  item1.req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(std::string("panda_link7"), pose_stamped));

  auto item2 = item1;
  item2.req.goal_constraints.clear();
  pose_stamped.pose.position.x = 0.49;
  pose_stamped.pose.position.y = -0.005;
  item2.req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(std::string("panda_link7"), pose_stamped));

  auto item3 = item1;
  item3.req.goal_constraints.clear();
  pose_stamped.pose.position.x = 0.475;
  pose_stamped.pose.position.y = -0.0075;
  item3.req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(std::string("panda_link7"), pose_stamped));

  auto service_request = std::make_shared<moveit_msgs::srv::GetMotionSequence::Request>();
  service_request->request.items.push_back(item1);
  service_request->request.items.push_back(item2);
  service_request->request.items.push_back(item3);

  auto service_future = service_client->async_send_request(service_request);
  auto service_response = service_future.get();

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  if (service_response->response.error_code.val == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    plan.trajectory_ = service_response.get()->response.planned_trajectories[0];
  }

  move_group.execute(plan);

  rclcpp::shutdown();
  return 0;
}