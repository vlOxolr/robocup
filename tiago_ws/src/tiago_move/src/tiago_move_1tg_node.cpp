#include <ros/ros.h>
#include <string>
#include <vector>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf/transform_datatypes.h>     // tf::createQuaternionMsgFromYaw
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>

#include <tiago_move/controller.h>

namespace tiago_move
{


Controller::Controller()
: body_planner_("arm_torso")          
, ac("move_base", /*spin_thread=*/true)   // Ex3：connect move_base action
{
}

Controller::~Controller() {}

bool Controller::initialize(ros::NodeHandle& /*nh*/)
{
  // 等待 move_base action server
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::NodeHandle pnh("~");

  // ------------- Ex3: 加载单个导航目标 -------------
  if (!pnh.getParam("target_position", target_position) || target_position.size() != 3) {
    ROS_ERROR("target_position not provided or size != 3");
    return false;
  } else {
    ROS_INFO("Loaded target_pose: [%.3f, %.3f, %.3f]",
             target_position[0], target_position[1], target_position[2]);
  }

  // ------------- Ex4: 机械臂目标位姿 -------------
  if (!pnh.getParam("target_pose", target_pose) || target_pose.size() != 6) {
    ROS_WARN("target_pose not provided or size != 6, skip arm motion in Ex4.");
  } else {
    ROS_INFO("Loaded target_pose.");
  }

  // 保留 MoveIt 设置
  body_planner_.setPlannerId("RRTConnectkConfigDefault");
  body_planner_.setPoseReferenceFrame("base_link");

  return true;
}

move_base_msgs::MoveBaseGoal Controller::createGoal(std::vector<double>& goal)
{
  move_base_msgs::MoveBaseGoal goal_msg;

  goal_msg.target_pose.header.frame_id = "map";
  goal_msg.target_pose.header.stamp = ros::Time::now();
  goal_msg.target_pose.pose.position.x = goal[0];
  goal_msg.target_pose.pose.position.y = goal[1];

  goal_msg.target_pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(goal[2]);

  return goal_msg;
}

// Uncomment the function for Exercise 4
int Controller::move_arm(std::vector<double> &goal)
{
  // input: std::vector<double> &goal [x,y,z,r,p,y]
  //#>>>>TODO:Exercise4 Create a msg of type geometry_msgs::PoseStamped from the input vector 
  geometry_msgs::PoseStamped target;
  target.header.stamp = ros::Time::now();
  target.header.frame_id = "base_link";
  target.pose.position.x = goal[0];
  target.pose.position.y = goal[1];
  target.pose.position.z = goal[2];

  tf::Quaternion q = tf::createQuaternionFromRPY(goal[3], goal[4], goal[5]);
  target.pose.orientation.x = q.x();
  target.pose.orientation.y = q.y();
  target.pose.orientation.z = q.z();
  target.pose.orientation.w = q.w();

  //#>>>>TODO:Exercise4 Set the target pose for the planner setPoseTarget function of your MoveGroupInterface instance
  body_planner_.setPoseTarget(target);

  ROS_INFO_STREAM("Planning to move "
                  << body_planner_.getEndEffectorLink()
                  << " to a target pose expressed in "
                  << body_planner_.getPlanningFrame());

  body_planner_.setStartStateToCurrentState();
  body_planner_.setPlannerId("RRTConnectkConfigDefault"); 
  body_planner_.setMaxVelocityScalingFactor(0.5);
  body_planner_.setPlanningTime(10.0);
  body_planner_.setNumPlanningAttempts(10);
  body_planner_.setGoalPositionTolerance(0.02);
  body_planner_.setGoalOrientationTolerance(0.05);
  body_planner_.setWorkspace(-1.0, -1.0, 0.0, 1.0, 1.0, 1.5);

  moveit::planning_interface::MoveGroupInterface::Plan motion_plan;

  //#>>>>TODO:Exercise4 Start the planning by calling member function "plan" and pass the motion_plan as argument
  bool ok = (body_planner_.plan(motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!ok) {
    ROS_WARN("MoveIt failed to find a plan.");
    return EXIT_FAILURE;
  }

  ROS_INFO_STREAM("Plan found in " << motion_plan.planning_time_ << " seconds");
  //#>>>>TODO:Exercise4 Execute the plan by calling member function "move" of the MoveGroupInterface instance
  auto exec_ok = body_planner_.move();
  if (exec_ok != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_WARN("MoveIt failed to execute plan.");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

} // namespace tiago_move

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tiago_move_1tg");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  tiago_move::Controller controller;
  if (!controller.initialize(nh)) {
    ROS_ERROR_STREAM("tiago_move::Controller failed to initialize");
    return -1;
  }

  // 检查 target_position 是否有效
  if (controller.target_position.size() != 3) {
    ROS_ERROR("Invalid target_position, need [x, y, yaw]");
    return -1;
  }

  // -----------------------------
  // 1. 导航到底盘目标 target_position
  // -----------------------------
  ROS_INFO("Sending navigation goal to target_position...");
  move_base_msgs::MoveBaseGoal goal_msg =
      controller.createGoal(controller.target_position);

  controller.ac.sendGoal(goal_msg);
  controller.ac.waitForResult();

  if (controller.ac.getState() ==
      actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Reached target_position.");

    // -----------------------------
    // 2. 可选：在目标点执行一次机械臂动作
    // -----------------------------
    if (!controller.target_pose.empty()) {
      ROS_INFO("Trigger arm motion at target_position ...");
      int ret = controller.move_arm(controller.target_pose);
      if (ret != EXIT_SUCCESS) {
        ROS_WARN("move_arm() failed.");
      }
    }
  }
  else
  {
    ROS_WARN("Failed to reach target_position, state = %s",
             controller.ac.getState().toString().c_str());
    spinner.stop();
    return -1;
  }

  ROS_INFO("tiago_move_1tg finished.");
  spinner.stop();
  return 0;
}