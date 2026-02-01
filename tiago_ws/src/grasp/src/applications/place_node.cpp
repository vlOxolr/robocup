#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tf/transform_datatypes.h>

namespace {
bool planToPose(moveit::planning_interface::MoveGroupInterface& move_group,
                const geometry_msgs::Pose& target_pose) {
  // 使用当前状态作为起点，避免状态不同步
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = move_group.plan(plan);
  move_group.clearPoseTargets();

  if (plan_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("规划失败");
    return false;
  }

  auto exec_result = move_group.execute(plan);
  if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("执行失败");
    return false;
  }

  return true;
}

bool cartesianToPose(moveit::planning_interface::MoveGroupInterface& move_group,
                     const geometry_msgs::Pose& target_pose,
                     double eef_step,
                     double jump_threshold) {
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(move_group.getCurrentPose().pose);
  waypoints.push_back(target_pose);

  moveit_msgs::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  if (fraction < 0.99) {
    ROS_ERROR("笛卡尔路径规划失败，完成比例: %.2f", fraction);
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  auto exec_result = move_group.execute(plan);
  if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("笛卡尔执行失败");
    return false;
  }

  return true;
}

bool moveEndEffectorStraightDirection(moveit::planning_interface::MoveGroupInterface& move_group,
                                      double dx, double dy, double dz,
                                      double distance,
                                      double eef_step,
                                      double jump_threshold) {
  Eigen::Vector3d dir(dx, dy, dz);
  if (dir.norm() < 1e-6) {
    ROS_ERROR("Direction vector is zero!");
    return false;
  }
  dir.normalize();

  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose(move_group.getEndEffectorLink());
  geometry_msgs::Pose start = current_pose.pose;
  geometry_msgs::Pose target = start;

  target.position.x += dir.x() * distance;
  target.position.y += dir.y() * distance;
  target.position.z += dir.z() * distance;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start);
  waypoints.push_back(target);

  moveit_msgs::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  if (fraction < 0.99) {
    ROS_ERROR("Cartesian path planning failed, fraction: %.2f", fraction);
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  auto exec_result = move_group.execute(plan);
  if (exec_result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("Cartesian execution failed");
    return false;
  }

  return true;
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "place_node");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // 基本参数
  std::string planning_group = "arm";
  std::string base_frame = "base_link";
  std::string ee_link = "gripper_link";
  std::string gripper_topic = "/gripper_controller/command";

  nh.param("planning_group", planning_group, planning_group);
  nh.param("base_frame", base_frame, base_frame);
  nh.param("ee_link", ee_link, ee_link);
  nh.param("gripper_topic", gripper_topic, gripper_topic);

  // 放置位置参数（base_frame 下）
  double place_x = 0.5;
  double place_y = 0.0;
  double place_z = 0.75;
  double place_roll = 0.0;
  double place_pitch = 0.0;
  double place_yaw = 0.0;
  nh.param("place_x", place_x, place_x);
  nh.param("place_y", place_y, place_y);
  nh.param("place_z", place_z, place_z);
  nh.param("place_roll", place_roll, place_roll);
  nh.param("place_pitch", place_pitch, place_pitch);
  nh.param("place_yaw", place_yaw, place_yaw);

  // 轨迹与流程参数
  double lift = 0.10;        // 先抬高的高度
  double pre_place = 0.10;   // 预放置高度（在放置点上方）
  double retreat = 0.10;     // 放置后上抬的高度
  double eef_step = 0.005;
  double jump_threshold = 0.0;
  nh.param("lift", lift, lift);
  nh.param("pre_place", pre_place, pre_place);
  nh.param("retreat", retreat, retreat);
  nh.param("eef_step", eef_step, eef_step);
  nh.param("jump_threshold", jump_threshold, jump_threshold);

  // 夹爪打开参数
  double gripper_open = 0.044;
  double gripper_wait = 1.0;
  nh.param("gripper_open", gripper_open, gripper_open);
  nh.param("gripper_wait", gripper_wait, gripper_wait);

  moveit::planning_interface::MoveGroupInterface move_group(planning_group);
  move_group.setPoseReferenceFrame(base_frame);
  move_group.setEndEffectorLink(ee_link);
  move_group.setPlanningTime(10.0);
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.1);

  // 桌面碰撞体（优先使用点云估计）
  bool enable_table = true;
  bool use_table_cloud = true;
  std::string table_cloud_topic = "/plane_segmentation/table_cloud";
  double table_box_height = 0.40;
  double table_z_offset = 0.08;
  double table_x_offset = 0.0;
  double table_y_margin = 0.0;
  nh.param("enable_table", enable_table, enable_table);
  nh.param("use_table_cloud", use_table_cloud, use_table_cloud);
  nh.param("table_cloud_topic", table_cloud_topic, table_cloud_topic);
  nh.param("table_box_height", table_box_height, table_box_height);
  nh.param("table_z_offset", table_z_offset, table_z_offset);
  nh.param("table_x_offset", table_x_offset, table_x_offset);
  nh.param("table_y_margin", table_y_margin, table_y_margin);

  // 固定桌面参数（当不使用点云时）
  double table_cx = 0.7;
  double table_cy = 0.0;
  double table_cz = 0.37;
  double table_sx = 0.8;
  double table_sy = 0.6;
  double table_sz = 0.05;
  nh.param("table_center_x", table_cx, table_cx);
  nh.param("table_center_y", table_cy, table_cy);
  nh.param("table_center_z", table_cz, table_cz);
  nh.param("table_size_x", table_sx, table_sx);
  nh.param("table_size_y", table_sy, table_sy);
  nh.param("table_size_z", table_sz, table_sz);

  // 将桌子加入规划场景，避免从下方绕行
  if (enable_table) {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::CollisionObject table;
    table.id = "place_table";
    table.header.frame_id = base_frame;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;

    if (use_table_cloud) {
      // 从平面分割点云估计桌子大小与位置
      sensor_msgs::PointCloud2ConstPtr msg =
          ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
              table_cloud_topic, nh, ros::Duration(2.0));
      if (msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (!cloud->empty()) {
          pcl::PointXYZ min_pt, max_pt;
          pcl::getMinMax3D(*cloud, min_pt, max_pt);

          double box_x = max_pt.x - min_pt.x;
          double box_y = (max_pt.y - min_pt.y) + 2.0 * table_y_margin;
          double box_z = table_box_height;

          double center_x = (min_pt.x + max_pt.x) / 2.0 + table_x_offset;
          double center_y = (min_pt.y + max_pt.y) / 2.0;
          double center_z = max_pt.z - box_z + table_z_offset;

          primitive.dimensions = {box_x, box_y, box_z};
          pose.position.x = center_x;
          pose.position.y = center_y;
          pose.position.z = center_z;

          ROS_INFO("Place table from cloud: center(%.3f, %.3f, %.3f) size(%.3f, %.3f, %.3f)",
                   center_x, center_y, center_z, box_x, box_y, box_z);
        } else {
          ROS_WARN("Table cloud is empty, fallback to fixed table parameters.");
          primitive.dimensions = {table_sx, table_sy, table_sz};
          pose.position.x = table_cx + table_x_offset;
          pose.position.y = table_cy;
          pose.position.z = table_cz;
          ROS_INFO("Place table fixed: center(%.3f, %.3f, %.3f) size(%.3f, %.3f, %.3f)",
                   table_cx, table_cy, table_cz, table_sx, table_sy, table_sz);
        }
      } else {
        ROS_WARN("No table cloud received, fallback to fixed table parameters.");
        primitive.dimensions = {table_sx, table_sy, table_sz};
        pose.position.x = table_cx + table_x_offset;
        pose.position.y = table_cy;
        pose.position.z = table_cz;
        ROS_INFO("Place table fixed: center(%.3f, %.3f, %.3f) size(%.3f, %.3f, %.3f)",
                 table_cx, table_cy, table_cz, table_sx, table_sy, table_sz);
      }
    } else {
      primitive.dimensions = {table_sx, table_sy, table_sz};
      pose.position.x = table_cx + table_x_offset;
      pose.position.y = table_cy;
      pose.position.z = table_cz;
      ROS_INFO("Place table fixed: center(%.3f, %.3f, %.3f) size(%.3f, %.3f, %.3f)",
               table_cx, table_cy, table_cz, table_sx, table_sy, table_sz);
    }

    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(pose);
    table.operation = table.ADD;

    planning_scene_interface.applyCollisionObject(table);
  }

  ros::Publisher gripper_pub =
      nh.advertise<trajectory_msgs::JointTrajectory>(gripper_topic, 1, true);

  // 计算放置位姿
  tf::Quaternion q;
  q.setRPY(place_roll, place_pitch, place_yaw);
  geometry_msgs::Pose place_pose;
  place_pose.position.x = place_x;
  place_pose.position.y = place_y;
  place_pose.position.z = place_z;
  place_pose.orientation.x = q.x();
  place_pose.orientation.y = q.y();
  place_pose.orientation.z = q.z();
  place_pose.orientation.w = q.w();

  // 1) 移动到放置点上方（与 grasp.cpp 一致：先规划到预放置位）
  geometry_msgs::Pose pre_place_pose = place_pose;
  pre_place_pose.position.z += pre_place;
  ROS_INFO("Pre-place pose: pos(%.3f, %.3f, %.3f) ori(%.3f, %.3f, %.3f, %.3f)",
           pre_place_pose.position.x, pre_place_pose.position.y, pre_place_pose.position.z,
           pre_place_pose.orientation.x, pre_place_pose.orientation.y,
           pre_place_pose.orientation.z, pre_place_pose.orientation.w);
  if (!planToPose(move_group, pre_place_pose)) {
    return 1;
  }

  // 1.5) 执行调整移动 (0.05, 0, 0.03)
  ROS_INFO("Adjusting position by (0.1, 0, 0.03)");
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose(move_group.getEndEffectorLink());
  geometry_msgs::Pose adjust_pose = current_pose.pose;
  adjust_pose.position.x += 0.17;
  adjust_pose.position.y += 0.0;
  adjust_pose.position.z += 0.0;

  if (!cartesianToPose(move_group, adjust_pose, eef_step, jump_threshold)) {
    ROS_ERROR("Adjustment move failed");
    return 1;
  }

  // 2) 直线下降到放置点（笛卡尔）
  // 由于步骤1.5向上移动了0.03m，需要额外下降0.03m
 // double total_descent = pre_place + ;
    // 2) 直线下降到放置点（笛卡尔）
  if (pre_place > 1e-4) {
    if (!moveEndEffectorStraightDirection(move_group, 0, 0, -0.03, pre_place, eef_step, jump_threshold)) {
      return 1;
    }
  }

  // 3) 打开夹爪释放物体
  trajectory_msgs::JointTrajectory traj;
  traj.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};
  trajectory_msgs::JointTrajectoryPoint pt;
  pt.positions = {gripper_open, gripper_open};
  pt.time_from_start = ros::Duration(1.0);
  traj.points.push_back(pt);
  traj.header.stamp = ros::Time::now() + ros::Duration(0.1);
  gripper_pub.publish(traj);
  ros::Duration(gripper_wait).sleep();

  // 4) 放置完成后上抬退出（笛卡尔）
  if (retreat > 1e-4) {
    if (!moveEndEffectorStraightDirection(move_group, 0, 0, 1, retreat, eef_step, jump_threshold)) {
      return 1;
    }
  }

  ROS_INFO("放置完成");
  return 0;
}