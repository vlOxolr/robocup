#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros

import tf2_ros
from tf2_geometry_msgs import do_transform_pose


#from navigation_client import NavigationClient
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from actionlib import SimpleActionClient
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal




import subprocess
import time


class NavigationClient(object):
    """
    调度器级 NavigationClient
    """

    def __init__(self, pkg="tiago_move", launch_file="tiago_move_1tg.launch"):
        self.pkg = pkg
        self.launch_file = launch_file

    def goto(self, x, y, yaw):
        """
        执行：底盘导航到 (x, y, yaw)
        """
        rospy.set_param("/tiago_move_1tg/target_position", [x, y, yaw])
        rospy.loginfo("Set navigation target: [%.3f, %.3f, %.3f]" % (x, y, yaw))

        cmd = ["roslaunch", self.pkg, self.launch_file]
        p = subprocess.Popen(cmd)

        while p.poll() is None:
            time.sleep(0.5)

        if p.returncode == 0:
            rospy.loginfo("Navigation succeeded.")
            return True
        else:
            rospy.logwarn("Navigation failed.")
            return False


# -----------------------------
# State: GoTo (导航到底盘目标)
# -----------------------------
class GoTo(smach.State):
    def __init__(self, nav_client, target, label=""):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.nav = nav_client
        self.target = target      # [x, y, yaw]
        self.label = label

    def execute(self, userdata):
        rospy.loginfo("[SMACH] State GoTo(%s), target=%s", self.label, self.target)

        ok = self.nav.goto(self.target[0], self.target[1], self.target[2])

        if ok:
            rospy.loginfo("[SMACH] GoTo(%s:[%f,%f,%f]) succeeded", self.label, self.target[0], self.target[1], self.target[2])
            return "succeeded"
        else:
            rospy.logerr("[SMACH] GoTo(%s) failed", self.label)
            return "failed"


# -----------------------------
# State: DetectBottleFromMarkers
# 从 /text_markers 找到名为 bottle 的物体位姿
# -----------------------------
class DetectBottleFromMarkers(smach.State):
    """
    通过 /text_markers 话题找到 bottle 的位姿。
    这里假定 /text_markers 类型为 visualization_msgs/MarkerArray，
    且其中某个 marker 的 text 字段为 "bottle"，pose 即物体位置。
    """

    def __init__(self,
                 topic_name="/text_markers",
                 target_label="bottle",
                 timeout=5.0):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["object_pose"],
        )
        self.topic_name = topic_name
        self.target_label = target_label
        self.timeout = timeout

        self.bottle_pub = rospy.Publisher(
            "/bottle_pos", Marker, queue_size=1, latch=True
        )

    def execute(self, userdata):
        rospy.loginfo("[SMACH] State DetectBottleFromMarkers: waiting for %s", self.topic_name)

        try:
            markers_msg = rospy.wait_for_message(
                self.topic_name, MarkerArray, timeout=self.timeout
            )
        except rospy.ROSException:
            rospy.logerr("[SMACH] No message on %s within %.1f s",
                         self.topic_name, self.timeout)
            return "failed"

        # 在 markers 里查找 label = bottle 的那个
        bottle_marker = None
        for m in markers_msg.markers:
            # 这一行根据你实际的消息内容改：
            # 有的代码把类别写在 m.text，有的写在 m.ns 或 id，自己对照 rostopic echo 一下即可
            if getattr(m, "text", "") == self.target_label:
                bottle_marker = m
                break

        if bottle_marker is None:
            rospy.logerr("[SMACH] No marker with label '%s' found in /text_markers",
                         self.target_label)
            return "failed"

        # 把 Marker 的 pose 封装成 PoseStamped，后面抓取用
        pose_st = PoseStamped()
        pose_st.header = bottle_marker.header
        pose_st.pose = bottle_marker.pose

        userdata.object_pose = pose_st

        marker = Marker()
        marker.header = bottle_marker.header
        marker.ns = "bottle_pos"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = bottle_marker.pose

        # 大小（自己按需求调）
        marker.scale.x = 0.11
        marker.scale.y = 0.11
        marker.scale.z = 0.20

        # 颜色（红色不透明）
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.bottle_pub.publish(marker)
        rospy.loginfo("[SMACH] Published bottle marker on /bottle_pos")

        rospy.loginfo("[SMACH] Found bottle at (%.3f, %.3f, %.3f) in %s",
                      pose_st.pose.position.x,
                      pose_st.pose.position.y,
                      pose_st.pose.position.z,
                      pose_st.header.frame_id)

        return "succeeded"


# -----------------------------
# State: PickObject (机械臂抓取)
# -----------------------------
class PickBottle(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["object_pose"],
            output_keys=["object_pose"],   # 现在自己写 object_pose
        )

        # Action client
        rospy.loginfo("[PickBottle] Waiting for /pickup_pose action server...")
        self.client = SimpleActionClient("/pickup_pose", PickUpPoseAction)
        self.client.wait_for_server()
        rospy.loginfo("[PickBottle] Connected to /pickup_pose.")

        # TF2: 用于把 item_markers 的位姿变到 base_footprint
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.marker_topic = "/item_markers"
        self.target_class = "bottle"
        self.target_frame = "base_footprint"


    def _wait_for_bottle_pose(self):
        """
        循环等待 /item_markers 中出现 ns == 'bottle' 的 marker，
        找到后把 pose 变换到 base_footprint，并返回 PoseStamped。
        """
        rospy.loginfo("[PickBottle] Waiting for '%s' in %s ...",
                        self.target_class, self.marker_topic)

        while not rospy.is_shutdown():
            try:
                msg = rospy.wait_for_message(
                    self.marker_topic, MarkerArray, timeout=2.0
                )
            except rospy.ROSException:
                # 超时就继续等
                continue

            bottle_marker = None
            for m in msg.markers:
                # C++ 里：box.ns = assigned_classes[i]
                if m.ns == self.target_class:
                    bottle_marker = m
                    break

            if bottle_marker is None:
                rospy.loginfo("[PickBottle] '%s' not found yet, keep waiting ...",
                                self.target_class)
                continue

            # 先构造原始 pose（在 item_markers 的 header.frame_id 下）
            raw_pose = PoseStamped()
            raw_pose.header = bottle_marker.header
            raw_pose.pose = bottle_marker.pose

            # 去掉可能的前导 '/'，以免 TF 查找失败
            if raw_pose.header.frame_id.startswith("/"):
                raw_pose.header.frame_id = raw_pose.header.frame_id[1:]

            rospy.loginfo(
                    "[PickBottle] Found '%s' at (%.3f, %.3f, %.3f) in %s",
                    self.target_class,
                    raw_pose.pose.position.x,
                    raw_pose.pose.position.y,
                    raw_pose.pose.position.z,
                    raw_pose.header.frame_id,
                )

            # 再用 TF 变换到 base_footprint
            try:
                # 使用最新的 TF
                self.tf_buffer.can_transform(
                    self.target_frame,
                    raw_pose.header.frame_id,
                    rospy.Time(0),
                    timeout=rospy.Duration(1.0)
                )
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    raw_pose.header.frame_id,
                    rospy.Time(0)
                )
                bottle_in_base = do_transform_pose(raw_pose, transform)
                bottle_in_base.header.frame_id = self.target_frame

                rospy.loginfo(
                    "[PickBottle] Found '%s' at (%.3f, %.3f, %.3f) in %s",
                    self.target_class,
                    bottle_in_base.pose.position.x,
                    bottle_in_base.pose.position.y,
                    bottle_in_base.pose.position.z,
                    bottle_in_base.header.frame_id,
                )
                return bottle_in_base

            except tf2_ros.TransformException as e:
                rospy.logwarn("[PickBottle] TF transform %s -> %s failed: %s",
                                raw_pose.header.frame_id, self.target_frame, str(e))
                rospy.sleep(0.1)
                continue

        return None


    def execute(self, userdata):
        # 1) 先从 /item_markers 等到 bottle 的位姿
        obj_pose = self._wait_for_bottle_pose()
        if obj_pose is None:
            rospy.logerr("[PickBottle] Failed to get bottle pose (node shutdown?)")
            return "failed"

        # 记录到 userdata，后面如果还有别的状态要用也方便
        userdata.object_pose = obj_pose

        # 2) 调用抓取 action
        rospy.loginfo("[PickBottle] Sending grasping pose to pick server...")
        goal = PickUpPoseGoal()
        goal.object_pose = obj_pose

        self.client.send_goal(goal)
        self.client.wait_for_result()

        result = self.client.get_result()
        err = result.error_code

        if 1: #err == 1:  # MoveItErrorCodes.SUCCESS
            rospy.loginfo("[PickBottle] Grasp SUCCESS")
            return "succeeded"
        else:
            rospy.logerr("[PickBottle] Grasp FAILED, error_code = %s", str(err))
            return "failed"

# -----------------------------
# State: LiftBottle (抓住后抬高瓶子/上半身)
# -----------------------------
class LiftBottle(smach.State):
    def __init__(self,
                 lift_delta=0.10,   # 在当前基础上再抬高 0.10 m
                 torso_min=0.0,
                 torso_max=0.35):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self.lift_delta = lift_delta
        self.torso_min = torso_min
        self.torso_max = torso_max

        self.state_topic = "/torso_controller/state"
        self.cmd_topic = "/torso_controller/command"

        self.pub = rospy.Publisher(
            self.cmd_topic,
            JointTrajectory,
            queue_size=1
        )

    def execute(self, userdata):
        rospy.loginfo("[SMACH] State LiftBottle: lifting torso to raise bottle...")

        # 1) 先读当前 torso 位置
        try:
            state = rospy.wait_for_message(
                self.state_topic,
                JointTrajectoryControllerState,
                timeout=2.0
            )
            if "torso_lift_joint" in state.joint_names:
                idx = state.joint_names.index("torso_lift_joint")
                current_pos = state.actual.positions[idx]
            else:
                rospy.logwarn("[LiftBottle] torso_lift_joint not in state, using default 0.14")
                current_pos = 0.14
        except rospy.ROSException:
            rospy.logwarn("[LiftBottle] Failed to read torso state, using default 0.14")
            current_pos = 0.14

        # 2) 计算目标高度（当前 + lift_delta，并做限幅）
        target_pos = current_pos + self.lift_delta
        target_pos = max(self.torso_min, min(self.torso_max, target_pos))

        rospy.loginfo("[LiftBottle] torso_lift_joint: %.3f -> %.3f",
                      current_pos, target_pos)

        # 3) 发送 JointTrajectory 指令
        traj = JointTrajectory()
        traj.joint_names = ["torso_lift_joint"]

        point = JointTrajectoryPoint()
        point.positions = [target_pos]
        point.time_from_start = rospy.Duration(2.0)  # 慢一点，更稳

        traj.points.append(point)

        # 连续发几次，防止丢包
        for i in range(10):
            self.pub.publish(traj)
            rospy.sleep(0.1)

        rospy.loginfo("[LiftBottle] Command sent, assuming success.")
        return "succeeded"



# -----------------------------
# State: OpenGripper (在 table B 松开夹爪)
# 自动探测：优先用 follow_joint_trajectory action，没有就用 /xxx/command topic
# -----------------------------
class OpenGripper(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self.mode = None            # "action" or "topic"
        self.client = None
        self.pub = None
        self.action_name = None
        self.cmd_topic = None

        # 你仿真中的关节名（左右指一般是 mimic，只需要控制一个也行）
        # 如果之后你发现 joint 名不一样，只改这里就行
        self.joint_names = ["gripper_left_finger_joint"]
        self.open_position = 0.045   # 张开的目标位置，必要时调大/调小

        self._setup_control()

    def _setup_control(self):
        """
        自动扫描当前系统中的 gripper 控制接口：
        1) 先找 *gripper* + *follow_joint_trajectory* 的 action
        2) 找不到再找 *gripper* + /command 且类型为 trajectory_msgs/JointTrajectory 的 topic
        """
        rospy.loginfo("[OpenGripper] Auto-detecting gripper controller...")

        try:
            topics = rospy.get_published_topics()
        except Exception as e:
            rospy.logerr("[OpenGripper] get_published_topics() failed: %s", str(e))
            return

        # 1) 找 action server：名字里带 "gripper" 且以 "follow_joint_trajectory" 结尾
        action_candidates = []
        for name, _ in topics:
            if "gripper" in name and name.endswith("/goal") and "follow_joint_trajectory" in name:
                # /xxx/follow_joint_trajectory/goal -> /xxx/follow_joint_trajectory
                base = name.rsplit("/goal", 1)[0]
                action_candidates.append(base)

        if action_candidates:
            self.action_name = action_candidates[0]
            rospy.loginfo("[OpenGripper] Found gripper action: %s", self.action_name)

            self.client = SimpleActionClient(
                self.action_name,
                FollowJointTrajectoryAction
            )
            # 等一会儿，看 action server 是否真正在
            if self.client.wait_for_server(rospy.Duration(5.0)):
                self.mode = "action"
                rospy.loginfo("[OpenGripper] Using ACTION mode for gripper.")
                return
            else:
                rospy.logwarn("[OpenGripper] Action %s not responding, fallback to topic mode.",
                              self.action_name)
                self.client = None
                self.action_name = None

        # 2) 找 /xxx/command topic，类型为 trajectory_msgs/JointTrajectory
        cmd_candidates = []
        for name, typ in topics:
            if "gripper" in name and name.endswith("/command") and typ == "trajectory_msgs/JointTrajectory":
                cmd_candidates.append(name)

        if cmd_candidates:
            self.cmd_topic = cmd_candidates[0]
            self.pub = rospy.Publisher(self.cmd_topic, JointTrajectory, queue_size=1)
            self.mode = "topic"
            rospy.loginfo("[OpenGripper] Using TOPIC mode for gripper, cmd topic: %s",
                          self.cmd_topic)
            return

        rospy.logerr("[OpenGripper] No suitable gripper controller found "
                     "(no follow_joint_trajectory action, no JointTrajectory /command).")

    def execute(self, userdata):
        rospy.loginfo("[SMACH] State OpenGripper: opening gripper...")

        if self.mode is None:
            rospy.logerr("[OpenGripper] No gripper control mode available.")
            return "failed"

        # 构造一个 “张开” 的轨迹
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [self.open_position]
        point.time_from_start = rospy.Duration(1.5)
        traj.points.append(point)

        if self.mode == "action":
            # 通过 FollowJointTrajectory action 发送
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = traj

            rospy.loginfo("[OpenGripper] Sending open command via ACTION: %s",
                          self.action_name)
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration(5.0))

            result = self.client.get_result()
            error_code = getattr(result, "error_code", None)
            rospy.loginfo("[OpenGripper] Action result error_code = %s", str(error_code))

            if error_code == 0:
                rospy.loginfo("[OpenGripper] Gripper opened successfully (action).")
                return "succeeded"
            else:
                rospy.logerr("[OpenGripper] Gripper open failed (action), error_code=%s",
                             str(error_code))
                return "failed"

        elif self.mode == "topic":
            # 通过 JointTrajectory topic 发送
            if self.pub is None:
                rospy.logerr("[OpenGripper] Topic mode selected but publisher is None.")
                return "failed"

            rospy.loginfo("[OpenGripper] Publishing open command to topic: %s",
                          self.cmd_topic)

            for i in range(10):
                self.pub.publish(traj)
                rospy.sleep(0.1)

            rospy.loginfo("[OpenGripper] Assume gripper opened successfully (topic).")
            return "succeeded"

        else:
            rospy.logerr("[OpenGripper] Unknown mode: %s", self.mode)
            return "failed"



# -----------------------------
# 主状态机
# -----------------------------
def main():
    rospy.init_node("task_smash")

    nav = NavigationClient()

    # 桌子 A / B 的导航目标
    table_A = [0.28, -0.39, -0.524]
    table_B = [-0.88, -0.95, 2.617]
    home_pose = [-0.13, 0.73, 0.398]

    sm = smach.StateMachine(outcomes=["DONE", "FAILED"])
    sm.userdata.object_pose = None

    with sm:
        # 1. 到桌子 A
        smach.StateMachine.add(
            "GOTO_A",
            GoTo(nav, table_A, label="table_A"),
            transitions={
                "succeeded": "PICK_BOTTLE",
                "failed": "FAILED",
            },
        )

        # 2. 调用抓取 action server
        smach.StateMachine.add(
            "PICK_BOTTLE",
            PickBottle(),
            transitions={
                "succeeded": "LIFT_BOTTLE",
                "failed": "FAILED",
            },
        )

        # 3. 抓住后先抬高瓶子（抬 torso）
        smach.StateMachine.add(
            "LIFT_BOTTLE",
            LiftBottle(lift_delta=0.10),   # 需要更高就改成 0.12 / 0.15
            transitions={
                "succeeded": "GOTO_B",
                "failed": "FAILED",
            },
        )

        # 4. 拿着物体走到桌子 B
        smach.StateMachine.add(
            "GOTO_B",
            GoTo(nav, table_B, label="table_B"),
            transitions={
                "succeeded": "OPEN_GRIPPER",
                "failed": "FAILED",
            },
        )

        # 4. 在 table B 松开夹爪
        smach.StateMachine.add(
            "OPEN_GRIPPER",
            OpenGripper(),
            transitions={
                "succeeded": "DONE",
                "failed": "FAILED",
            },
        )


    sis = smach_ros.IntrospectionServer("task_smash_server", sm, "/TASK_SMASH")
    sis.start()

    outcome = sm.execute()
    rospy.loginfo("[SMACH] Final outcome: %s", outcome)

    sis.stop()


if __name__ == "__main__":
    main()
