// ROS headers
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <teleop_tools_msgs/IncrementActionGoal.h>

// Std C++ headers
#include <string>
#include <vector>
#include <cmath>
#include <limits>

// ------------------------- Global state -------------------------
geometry_msgs::PoseStamped g_pregrasp_pose;
geometry_msgs::PoseStamped g_grasp_pose;
geometry_msgs::PoseStamped g_lift_pose;

bool g_has_target = false;
std::string g_target_frame = "base_link";

// ------------------------- Helpers -------------------------
static geometry_msgs::Quaternion rpyDegToQuat(double roll_deg, double pitch_deg, double yaw_deg)
{
  double roll = roll_deg  * M_PI / 180.0;
  double pitch = pitch_deg * M_PI / 180.0;
  double yaw = yaw_deg   * M_PI / 180.0;

  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::Quaternion quat;
  tf::quaternionTFToMsg(q, quat);
  return quat;
}

static bool adjustTorsoHeight(double increment)
{
  ros::NodeHandle nh;
  ros::Publisher torsoPub =
      nh.advertise<teleop_tools_msgs::IncrementActionGoal>("/torso_controller/increment/goal", 1, true);

  ros::Duration(0.3).sleep();

  if (torsoPub.getNumSubscribers() == 0)
  {
    ROS_ERROR("No subscribers on /torso_controller/increment/goal! (torso controller not running?)");
    return false;
  }

  teleop_tools_msgs::IncrementActionGoal msg;
  msg.goal.increment_by.push_back(increment);

  ROS_INFO_STREAM("Adjusting torso height by " << increment << " m");
  torsoPub.publish(msg);

  ros::Duration(1.0).sleep();
  return true;
}

static bool publishGripper(ros::Publisher& pub,
                           const std::vector<std::string>& joint_names,
                           const std::vector<double>& positions,
                           double time_sec = 1.0)
{
  if (joint_names.size() != positions.size() || joint_names.empty())
  {
    ROS_ERROR("Gripper joint_names and positions size mismatch (or empty).");
    return false;
  }

  trajectory_msgs::JointTrajectory traj;
  traj.joint_names = joint_names;

  trajectory_msgs::JointTrajectoryPoint pt;
  pt.positions = positions;
  pt.time_from_start = ros::Duration(time_sec);
  traj.points.push_back(pt);

  pub.publish(traj);
  return true;
}

static int moveArmToTarget(const geometry_msgs::PoseStamped& target,
                           const std::string& group_name,
                           const std::string& ee_link,
                           const std::string& planner_id,
                           double goal_tol,
                           double vel_scale,
                           double plan_time)
{
  moveit::planning_interface::MoveGroupInterface group(group_name);
  group.setPlannerId(planner_id);
  group.setPoseReferenceFrame(target.header.frame_id);
  group.setEndEffectorLink(ee_link);
  group.setGoalTolerance(goal_tol);
  group.setMaxVelocityScalingFactor(vel_scale);
  group.setPlanningTime(plan_time);
  group.setStartStateToCurrentState();
  group.setPoseTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (group.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_ERROR("Planning failed for target pose.");
    return EXIT_FAILURE;
  }

  if (group.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_ERROR("Failed to execute the planned motion.");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

// ------------------------- Marker selection -------------------------
static bool selectMarkerByNs(const visualization_msgs::MarkerArray& msg,
                            const std::string& target_ns,
                            visualization_msgs::Marker& out)
{
  for (const auto& m : msg.markers)
  {
    if (m.ns == target_ns)
    {
      out = m;
      return true;
    }
  }
  return false;
}

static bool selectNearestMarker(const visualization_msgs::MarkerArray& msg,
                                visualization_msgs::Marker& out)
{
  double best = std::numeric_limits<double>::infinity();
  bool found = false;
  for (const auto& m : msg.markers)
  {
    // 距离用 base_link 平面距离（x,y）
    const double d = std::sqrt(m.pose.position.x * m.pose.position.x +
                              m.pose.position.y * m.pose.position.y);
    if (d < best)
    {
      best = d;
      out = m;
      found = true;
    }
  }
  return found;
}

// ------------------------- Callback -------------------------
void markerCallback(const visualization_msgs::MarkerArray& msg)
{
  if (msg.markers.empty())
  {
    ROS_WARN_THROTTLE(1.0, "Received empty MarkerArray!");
    return;
  }

  ros::NodeHandle nh("~");

  // ---- target selection params ----
  std::string target_ns;
  nh.param<std::string>("target_ns", target_ns, std::string("apple"));  // 默认抓 apple

  std::string selection_mode;
  nh.param<std::string>("selection_mode", selection_mode, std::string("ns")); // "ns" or "nearest"

  // ---- offsets ----
  // 预抓取偏移（相对目标点）
  double pre_dx, pre_dy, pre_dz;
  nh.param("pregrasp_offset/x", pre_dx, 0.0);
  nh.param("pregrasp_offset/y", pre_dy, 0.0);
  nh.param("pregrasp_offset/z", pre_dz, 0.15); // 默认在上方 15cm

  // 抓取偏移（相对目标点）
  double gra_dx, gra_dy, gra_dz;
  nh.param("grasp_offset/x", gra_dx, 0.0);
  nh.param("grasp_offset/y", gra_dy, 0.0);
  nh.param("grasp_offset/z", gra_dz, 0.03); // 默认比物体中心高 3cm（避免撞桌）

  // 抬起偏移（相对抓取点）
  double lift_dx, lift_dy, lift_dz;
  nh.param("lift_offset/x", lift_dx, 0.0);
  nh.param("lift_offset/y", lift_dy, 0.0);
  nh.param("lift_offset/z", lift_dz, 0.10); // 默认抬起 10cm

  // ---- orientation fixed RPY (deg) ----
  double roll_deg, pitch_deg, yaw_deg;
  nh.param("ee_rpy/roll",  roll_deg,  -90.0);
  nh.param("ee_rpy/pitch", pitch_deg,   0.0);
  nh.param("ee_rpy/yaw",   yaw_deg,   180.0);
  geometry_msgs::Quaternion quat = rpyDegToQuat(roll_deg, pitch_deg, yaw_deg);

  // ---- select marker ----
  visualization_msgs::Marker target;
  bool found = false;

  if (selection_mode == "nearest")
    found = selectNearestMarker(msg, target);
  else
    found = selectMarkerByNs(msg, target_ns, target);

  if (!found)
  {
    ROS_WARN_THROTTLE(1.0, "Target not found. selection_mode=%s target_ns=%s (markers=%zu)",
                      selection_mode.c_str(), target_ns.c_str(), msg.markers.size());
    return;
  }

  g_target_frame = target.header.frame_id.empty() ? std::string("base_link") : target.header.frame_id;

  // ---- build poses ----
  const auto& p = target.pose.position;

  g_pregrasp_pose.header.frame_id = g_target_frame;
  g_pregrasp_pose.header.stamp = ros::Time::now();
  g_pregrasp_pose.pose.position.x = p.x + pre_dx;
  g_pregrasp_pose.pose.position.y = p.y + pre_dy;
  g_pregrasp_pose.pose.position.z = p.z + pre_dz;
  g_pregrasp_pose.pose.orientation = quat;

  g_grasp_pose.header.frame_id = g_target_frame;
  g_grasp_pose.header.stamp = ros::Time::now();
  g_grasp_pose.pose.position.x = p.x + gra_dx;
  g_grasp_pose.pose.position.y = p.y + gra_dy;
  g_grasp_pose.pose.position.z = p.z + gra_dz;
  g_grasp_pose.pose.orientation = quat;

  g_lift_pose.header.frame_id = g_target_frame;
  g_lift_pose.header.stamp = ros::Time::now();
  g_lift_pose.pose.position.x = g_grasp_pose.pose.position.x + lift_dx;
  g_lift_pose.pose.position.y = g_grasp_pose.pose.position.y + lift_dy;
  g_lift_pose.pose.position.z = g_grasp_pose.pose.position.z + lift_dz;
  g_lift_pose.pose.orientation = quat;

  g_has_target = true;

  ROS_INFO_THROTTLE(1.0, "Target selected: ns=%s frame=%s pos=(%.3f, %.3f, %.3f)",
                    target.ns.c_str(), g_target_frame.c_str(), p.x, p.y, p.z);
}

// ------------------------- Main -------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // ---- params ----
  std::string markers_topic;
  pnh.param<std::string>("markers_topic", markers_topic, std::string("/item_markers"));

  std::string group_name, ee_link, planner_id;
  pnh.param<std::string>("move_group", group_name, std::string("arm"));          // 或 "arm_torso"
  pnh.param<std::string>("ee_link", ee_link, std::string("gripper_link"));
  pnh.param<std::string>("planner_id", planner_id, std::string("RRTConnectkConfigDefault"));

  double goal_tol, vel_scale, plan_time;
  pnh.param("goal_tolerance", goal_tol, 0.02);
  pnh.param("velocity_scaling", vel_scale, 0.3);
  pnh.param("planning_time", plan_time, 10.0);

  bool use_torso;
  pnh.param("use_torso", use_torso, false);

  // 保留你原来的参数名，兼容旧 YAML
  double z2, z3;
  nh.param("grasp_adjustment/position/z", z2, 0.0); // torso increment before grasp (optional)
  nh.param("lift_adjustment/position/z",  z3, 0.0); // torso increment for lift (optional)

  // gripper params
  bool use_system_open;
  pnh.param("use_system_open", use_system_open, true); // 兼容你们原来的 home_gripper.py

  std::vector<std::string> gripper_joints;
  pnh.param("gripper_joints", gripper_joints,
            std::vector<std::string>({"gripper_left_finger_joint", "gripper_right_finger_joint"}));

  std::vector<double> gripper_open_pos, gripper_close_pos;
  pnh.param("gripper_open_pos",  gripper_open_pos,  std::vector<double>({1.0, 1.0}));
  pnh.param("gripper_close_pos", gripper_close_pos, std::vector<double>({0.0, 0.0}));

  double gripper_time;
  pnh.param("gripper_time", gripper_time, 1.0);

  // ---- subscriber/publisher ----
  ros::Subscriber sub = nh.subscribe(markers_topic, 1, markerCallback);
  ros::Publisher gripperPub = nh.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command", 1, true);

  ROS_INFO("Subscribing markers: %s", markers_topic.c_str());
  ROS_INFO("Move group: %s | ee_link: %s | planner: %s", group_name.c_str(), ee_link.c_str(), planner_id.c_str());

  // ---- open gripper ----
  if (use_system_open)
  {
    ROS_INFO("Opening gripper using system call: home_gripper.py");
    system("rosrun pal_gripper_controller_configuration_gazebo home_gripper.py");
    ros::Duration(1.0).sleep();
  }
  else
  {
    ROS_INFO("Opening gripper via /gripper_controller/command");
    publishGripper(gripperPub, gripper_joints, gripper_open_pos, gripper_time);
    ros::Duration(gripper_time).sleep();
  }

  // ---- wait for target ----
  ROS_INFO("Waiting for target markers...");
  ros::Rate r(20);
  while (ros::ok() && !g_has_target)
  {
    ros::spinOnce();
    r.sleep();
  }
  if (!ros::ok())
  {
    spinner.stop();
    return 0;
  }

  // ---- optional torso adjustment before approach ----
  if (use_torso && std::abs(z2) > 1e-6)
  {
    if (!adjustTorsoHeight(z2))
    {
      spinner.stop();
      return EXIT_FAILURE;
    }
  }

  // ---- pregrasp ----
  ROS_INFO("Moving to PRE-GRASP...");
  if (moveArmToTarget(g_pregrasp_pose, group_name, ee_link, planner_id, goal_tol, vel_scale, plan_time) != EXIT_SUCCESS)
  {
    spinner.stop();
    return EXIT_FAILURE;
  }
  ros::Duration(0.5).sleep();

  // ---- grasp (approach) ----
  ROS_INFO("Moving to GRASP...");
  if (moveArmToTarget(g_grasp_pose, group_name, ee_link, planner_id, goal_tol, vel_scale, plan_time) != EXIT_SUCCESS)
  {
    spinner.stop();
    return EXIT_FAILURE;
  }
  ros::Duration(0.5).sleep();

  // ---- close gripper ----
  ROS_INFO("Closing gripper...");
  publishGripper(gripperPub, gripper_joints, gripper_close_pos, gripper_time);
  ros::Duration(gripper_time).sleep();

  // ---- optional torso lift ----
  if (use_torso && std::abs(z3) > 1e-6)
  {
    if (!adjustTorsoHeight(z3))
    {
      spinner.stop();
      return EXIT_FAILURE;
    }
  }

  // ---- lift pose ----
  ROS_INFO("Lifting...");
  if (moveArmToTarget(g_lift_pose, group_name, ee_link, planner_id, goal_tol, vel_scale, plan_time) != EXIT_SUCCESS)
  {
    spinner.stop();
    return EXIT_FAILURE;
  }

  ROS_INFO("Grasp sequence completed.");
  spinner.stop();
  return EXIT_SUCCESS;
}
