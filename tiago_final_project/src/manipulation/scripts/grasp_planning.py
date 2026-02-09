#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
改进版抓取规划器
- 添加目标稳定性确认（连续N帧）
- 添加目标锁定机制
- 添加桌子碰撞物体
- 保留原有的前方接近策略
"""

import rospy
import moveit_commander
import actionlib
from geometry_msgs.msg import Pose, Quaternion, PointStamped, PoseStamped
from visualization_msgs.msg import MarkerArray
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import tf.transformations as tft
import tf
import numpy as np
import sys
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


class ImprovedGraspPlanner:
    """改进版抓取规划器"""

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('improved_grasp_planner', anonymous=True)

        # ==================== MoveIt 参数 ====================
        self.move_group_name = rospy.get_param('~move_group', 'arm_torso')
        self.end_effector_link = rospy.get_param('~end_effector_link', 'gripper_link')
        self.reference_frame = rospy.get_param('~reference_frame', 'base_footprint')
        self.planner_id = rospy.get_param('~planner_id', 'RRTConnectkConfigDefault')

        # ==================== 目标确认参数 ====================
        self.confirm_frames = rospy.get_param('~confirm_frames', 1)          # 需要连续N帧确认（1帧即可）
        self.confirm_pos_eps = rospy.get_param('~confirm_pos_eps', 0.03)     # 位置变化阈值(m)
        self.confirm_timeout = rospy.get_param('~confirm_timeout', 0.5)      # 帧间超时(s)
        self.lock_target = rospy.get_param('~lock_target', True)             # 锁定后不再更新

        # ==================== 抓取偏移参数 ====================
        # 预抓取位置偏移（相对于物体）
        self.pregrasp_offset_x = rospy.get_param('~pregrasp_offset/x', 0)  # 物体侧方30cm
        self.pregrasp_offset_y = rospy.get_param('~pregrasp_offset/y', 0.04)  # 物体前方4cm
        self.pregrasp_offset_z = rospy.get_param('~pregrasp_offset/z', 0.10)   # 物体上方10cm

        # 抓取位置偏移
        self.grasp_offset_x = rospy.get_param('~grasp_offset/x', 0)       
        self.grasp_offset_y = rospy.get_param('~grasp_offset/y', 0.0)
        self.grasp_offset_z = rospy.get_param('~grasp_offset/z', 0.10)

        # 抬起位置偏移（相对于抓取位置）
        self.lift_offset_x = rospy.get_param('~lift_offset/x', 0.0)
        self.lift_offset_y = rospy.get_param('~lift_offset/y', 0.0)
        self.lift_offset_z = rospy.get_param('~lift_offset/z', 0.15)           # 抬起15cm

        # ==================== 碰撞物体参数 ====================
        self.add_table_collision = rospy.get_param('~add_table_collision', True)
        self.table_size_x = rospy.get_param('~table_size/x', 0.8)
        self.table_size_y = rospy.get_param('~table_size/y', 0.8)
        self.table_size_z = rospy.get_param('~table_size/z', 0.02)  # 薄桌面，避免干扰

        # ==================== 内部状态 ====================
        # 目标确认状态
        self.confirm_count = 0
        self.candidate_ns = ""
        self.candidate_frame = ""
        self.candidate_pos = None
        self.candidate_scale = None
        self.last_confirm_time = None

        # 锁定状态
        self.target_locked = False
        self.locked_position = None
        self.locked_frame = None
        self.locked_scale = None
        self.locked_orientation = None

        # TF listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1.0)

        # MoveIt 规划组 - 等待move_group启动
        rospy.loginfo("Waiting for move_group to start...")
        rospy.loginfo("Connecting to planning group: %s" % self.move_group_name)
        try:
            self.arm_group = moveit_commander.MoveGroupCommander(
                self.move_group_name,
                wait_for_servers=30.0  # 等待30秒
            )
            rospy.loginfo("Successfully connected to move_group")
        except RuntimeError as e:
            rospy.logerr("Failed to connect to move_group: %s" % str(e))
            rospy.logerr("Make sure move_group is running:")
            rospy.logerr("  roslaunch tiago_moveit_config move_group.launch")
            raise
        self.arm_group.set_planner_id(self.planner_id)
        self.arm_group.set_pose_reference_frame(self.reference_frame)
        self.arm_group.set_end_effector_link(self.end_effector_link)
        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_group.set_planning_time(10.0)
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_goal_position_tolerance(0.03)
        self.arm_group.set_goal_orientation_tolerance(0.1)

        # 规划场景接口（用于添加碰撞物体）
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1.0)  # 等待场景接口初始化

        # 夹爪控制客户端
        self.gripper_client = actionlib.SimpleActionClient(
            '/gripper_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("等待夹爪控制器...")
        self.gripper_client.wait_for_server(rospy.Duration(5.0))

        # 订阅 MarkerArray
        # 支持多个目标物体名称（列表）
        self.target_ns = rospy.get_param('~target_ns', ['apple', 'sports ball'])
        rospy.Subscriber('/item_markers', MarkerArray, self.marker_callback)

        rospy.loginfo("=" * 60)
        rospy.loginfo("改进版抓取规划器初始化完成")
        rospy.loginfo("  目标确认: 需要连续 %d 帧, 位置阈值 %.2f m", self.confirm_frames, self.confirm_pos_eps)
        rospy.loginfo("  目标锁定: %s", "启用" if self.lock_target else "禁用")
        rospy.loginfo("  碰撞物体: %s", "添加桌子" if self.add_table_collision else "禁用")
        rospy.loginfo("  目标物体: %s", self.target_ns)
        rospy.loginfo("=" * 60)

    # ==================== 工具函数 ====================

    def dist3(self, p1, p2):
        """计算两点之间的3D距离"""
        if p1 is None or p2 is None:
            return float('inf')
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        dz = p1[2] - p2[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def reset_confirmation(self):
        """重置确认状态"""
        self.confirm_count = 0
        self.candidate_ns = ""
        self.candidate_frame = ""
        self.candidate_pos = None
        self.candidate_scale = None
        self.last_confirm_time = None

    def transform_point(self, point, from_frame):
        """将点从 from_frame 转换到 base_footprint"""
        try:
            point_stamped = PointStamped()
            point_stamped.header.frame_id = from_frame
            point_stamped.header.stamp = rospy.Time(0)
            point_stamped.point.x = point[0]
            point_stamped.point.y = point[1]
            point_stamped.point.z = point[2]

            self.tf_listener.waitForTransform('base_footprint', from_frame,
                                              rospy.Time(0), rospy.Duration(4.0))
            point_in_base = self.tf_listener.transformPoint('base_footprint', point_stamped)

            return [point_in_base.point.x, point_in_base.point.y, point_in_base.point.z]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF 转换失败: %s" % str(e))
            return None

    # ==================== 目标确认回调 ====================

    def marker_callback(self, msg):
        """处理 MarkerArray，实现目标确认和锁定"""
        # 如果已锁定且启用锁定，忽略后续更新
        if self.lock_target and self.target_locked:
            return

        if not msg.markers:
            return

        # 查找目标 marker（支持单个名称或列表）
        target_marker = None
        target_names = self.target_ns if isinstance(self.target_ns, list) else [self.target_ns]
        for m in msg.markers:
            if m.ns in target_names:
                target_marker = m
                break

        if target_marker is None:
            # 目标消失，检查是否需要重置确认
            if self.last_confirm_time is not None:
                gap = (rospy.Time.now() - self.last_confirm_time).to_sec()
                if gap > self.confirm_timeout:
                    rospy.logwarn("目标消失超时，重置确认状态")
                    self.reset_confirmation()
            return

        # 提取位置和帧
        frame = target_marker.header.frame_id if target_marker.header.frame_id else "base_link"
        ns = target_marker.ns
        pos = [target_marker.pose.position.x,
               target_marker.pose.position.y,
               target_marker.pose.position.z]
        scale = [target_marker.scale.x, target_marker.scale.y, target_marker.scale.z]
        orientation = target_marker.pose.orientation

        now = rospy.Time.now()

        # 检查帧间超时
        if self.last_confirm_time is not None:
            gap = (now - self.last_confirm_time).to_sec()
            if gap > self.confirm_timeout:
                rospy.logwarn("帧间隔过大 (%.2fs)，重置确认", gap)
                self.reset_confirmation()

        # 第一次检测或目标变化 -> 重新开始确认
        if self.confirm_count == 0 or ns != self.candidate_ns or frame != self.candidate_frame:
            self.candidate_ns = ns
            self.candidate_frame = frame
            self.candidate_pos = pos
            self.candidate_scale = scale
            self.candidate_orientation = orientation
            self.confirm_count = 1
            self.last_confirm_time = now
            rospy.loginfo("候选目标: ns=%s frame=%s pos=(%.3f, %.3f, %.3f) 确认=1/%d",
                          ns, frame, pos[0], pos[1], pos[2], self.confirm_frames)
            return

        # 同一目标：检查位置稳定性
        dp = self.dist3(pos, self.candidate_pos)
        if dp <= self.confirm_pos_eps:
            self.confirm_count += 1
            self.candidate_pos = pos  # 更新为最新位置
            self.candidate_scale = scale
            self.candidate_orientation = orientation
            self.last_confirm_time = now

            if self.confirm_count < self.confirm_frames:
                rospy.loginfo_throttle(0.5, "确认中: ns=%s dp=%.3f 确认=%d/%d",
                                       ns, dp, self.confirm_count, self.confirm_frames)
                return

            # 达到确认帧数 -> 锁定目标
            self.target_locked = True
            self.locked_frame = self.candidate_frame
            self.locked_scale = self.candidate_scale
            self.locked_orientation = self.candidate_orientation

            # 转换到 base_footprint
            self.locked_position = self.transform_point(self.candidate_pos, self.locked_frame)

            if self.locked_position:
                rospy.loginfo("✅ 目标已锁定: ns=%s pos=(%.3f, %.3f, %.3f) 确认=%d帧",
                              self.candidate_ns,
                              self.locked_position[0],
                              self.locked_position[1],
                              self.locked_position[2],
                              self.confirm_frames)
            else:
                rospy.logerr("目标锁定失败：TF转换错误")
                self.target_locked = False

            self.reset_confirmation()
        else:
            # 位置不稳定 -> 重置
            rospy.logwarn_throttle(0.5, "目标不稳定 (dp=%.3f > eps=%.3f)，重置确认",
                                   dp, self.confirm_pos_eps)
            self.candidate_pos = pos
            self.candidate_scale = scale
            self.confirm_count = 1
            self.last_confirm_time = now

    # ==================== 碰撞物体 ====================

    def add_table_to_scene(self, object_z):
        """在物体下方添加桌子碰撞物体"""
        if not self.add_table_collision:
            return

        rospy.loginfo("添加桌子碰撞物体...")

        # 清除旧的碰撞物体
        self.scene.remove_world_object("table")
        self.scene.remove_world_object("part")
        rospy.sleep(0.5)

        # 计算桌面高度（物体底部）
        obj_height = self.locked_scale[2] if self.locked_scale else 0.12
        table_top_z = object_z - obj_height / 2.0

        # 添加桌子
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_footprint"
        table_pose.pose.position.x = self.locked_position[0]
        table_pose.pose.position.y = self.locked_position[1]
        table_pose.pose.position.z = table_top_z - self.table_size_z / 2.0
        table_pose.pose.orientation.w = 1.0

        self.scene.add_box("table", table_pose,
                           (self.table_size_x, self.table_size_y, self.table_size_z))

        rospy.loginfo("  桌子位置: (%.3f, %.3f, %.3f) 尺寸: (%.2f, %.2f, %.2f)",
                      table_pose.pose.position.x,
                      table_pose.pose.position.y,
                      table_pose.pose.position.z,
                      self.table_size_x, self.table_size_y, self.table_size_z)

        rospy.sleep(0.5)

    def remove_collision_objects(self):
        """清除碰撞物体"""
        self.scene.remove_world_object("table")
        self.scene.remove_world_object("part")
        rospy.sleep(0.3)

    # ==================== 夹爪控制 ====================

    def open_gripper(self):
        """打开夹爪"""
        rospy.loginfo("打开夹爪...")
        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [0.044, 0.044]
        point.velocities = [0.0, 0.0]
        point.time_from_start = rospy.Duration(2.0)

        traj.points.append(point)
        goal.trajectory = traj

        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result(rospy.Duration(3.0))
        rospy.loginfo("✓ 夹爪已打开")

    def close_gripper(self, position=0.008):
        """关闭夹爪"""
        rospy.loginfo("关闭夹爪到位置: %.3f", position)
        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.velocities = [0.0, 0.0]
        point.time_from_start = rospy.Duration(2.0)

        traj.points.append(point)
        goal.trajectory = traj

        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result(rospy.Duration(3.0))
        rospy.loginfo("✓ 夹爪已关闭")

    # ==================== 运动控制 ====================

    def move_to_pose(self, x, y, z, quat=None, roll=0, pitch=0, yaw=0):
        """移动到目标姿态"""
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_footprint"
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z

        if quat is not None:
            target_pose.pose.orientation = quat
        else:
            quat_val = tft.quaternion_from_euler(roll, pitch, yaw)
            target_pose.pose.orientation = Quaternion(*quat_val)

        rospy.loginfo("  目标位置: [%.3f, %.3f, %.3f]", x, y, z)

        self.arm_group.set_pose_target(target_pose)
        self.arm_group.set_start_state_to_current_state()

        rospy.loginfo("  规划中...")
        plan = self.arm_group.plan()

        if isinstance(plan, tuple):
            success = plan[0]
        else:
            success = len(plan.joint_trajectory.points) > 0

        if not success:
            rospy.logerr("  ✗ 规划失败")
            self.arm_group.clear_pose_targets()
            return False

        rospy.loginfo("  执行中...")
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        return success

    # ==================== 抓取流程 ====================

    def wait_for_target(self, timeout=30.0):
        """等待目标锁定"""
        rospy.loginfo("等待目标锁定 (需要连续 %d 帧稳定)...", self.confirm_frames)
        start = rospy.Time.now()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.target_locked and self.locked_position is not None:
                return True

            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed > timeout:
                rospy.logerr("等待目标超时 (%.1fs)", timeout)
                return False

            rate.sleep()

        return False

    def grasp_object(self):
        """完整抓取流程"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("开始抓取流程")
        rospy.loginfo("=" * 60)

        # 等待目标锁定
        if not self.wait_for_target():
            return False

        obj_x, obj_y, obj_z = self.locked_position
        scale = self.locked_scale if self.locked_scale else [0.06, 0.06, 0.12]
        bbox_w, bbox_d, bbox_h = scale

        rospy.loginfo("\n物体信息:")
        rospy.loginfo("  位置: [%.3f, %.3f, %.3f]", obj_x, obj_y, obj_z)
        rospy.loginfo("  尺寸: [%.1f, %.1f, %.1f] cm", bbox_w*100, bbox_d*100, bbox_h*100)

        # 添加桌子碰撞物体
        self.add_table_to_scene(obj_z)

        # 获取姿态（使用marker的orientation或默认）
        quat = self.locked_orientation if hasattr(self, 'locked_orientation') and self.locked_orientation else None

        # 计算抓取位置
        pre_x = obj_x + self.pregrasp_offset_x
        pre_y = obj_y + self.pregrasp_offset_y
        pre_z = obj_z + bbox_h / 2.0 + self.pregrasp_offset_z

        grasp_x = obj_x + self.grasp_offset_x
        grasp_y = obj_y + self.grasp_offset_y
        grasp_z = obj_z + self.grasp_offset_z

        lift_x = grasp_x + self.lift_offset_x
        lift_y = grasp_y + self.lift_offset_y
        lift_z = grasp_z + self.lift_offset_z

        # 1. 打开夹爪
        rospy.loginfo("\n[1/5] 打开夹爪")
        self.open_gripper()
        rospy.sleep(0.5)

        # 2. 移动到预抓取位置
        rospy.loginfo("\n[2/5] 移动到预抓取位置")
        if not self.move_to_pose(pre_x, pre_y, pre_z, quat=quat):
            rospy.logerr("预抓取位置移动失败")
            return False
        rospy.sleep(0.5)

        # 3. 移动到抓取位置
        rospy.loginfo("\n[3/5] 移动到抓取位置")
        if not self.move_to_pose(grasp_x, grasp_y, grasp_z, quat=quat):
            rospy.logerr("抓取位置移动失败")
            return False
        rospy.sleep(0.5)

        # 4. 关闭夹爪
        rospy.loginfo("\n[4/5] 关闭夹爪")
        self.close_gripper(position=0.008)
        rospy.sleep(0.5)

        # 5. 抬起
        rospy.loginfo("\n[5/5] 抬起物体")
        if not self.move_to_pose(lift_x, lift_y, lift_z, quat=quat):
            rospy.logerr("抬起失败")
            return False

        # 清除碰撞物体
        self.remove_collision_objects()

        rospy.loginfo("\n✅ 抓取完成!")
        return True


def main():
    planner = ImprovedGraspPlanner()
    rospy.sleep(2.0)

    success = planner.grasp_object()

    if success:
        rospy.loginfo("\n=== 抓取成功 ===")
    else:
        rospy.logwarn("\n=== 抓取失败 ===")

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("用户中断")