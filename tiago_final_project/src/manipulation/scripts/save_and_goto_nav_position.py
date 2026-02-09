#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
保存和导航到 TIAGo 的 2D 位置

功能:
1. 保存当前机器人的 2D 位置 (x, y, theta)
2. 使用 move_base 导航到保存的位置
"""

import rospy
import tf
import json
import os
import sys
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion


class NavPositionManager:
    """2D 导航位置管理器"""

    def __init__(self):
        rospy.init_node('nav_position_manager', anonymous=True)

        # TF 监听器
        self.tf_listener = tf.TransformListener()

        # move_base 客户端
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待 move_base 服务器...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("✓ move_base 已连接")

        # 位置文件路径
        self.positions_file = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'saved_nav_positions.json'
        )

        rospy.loginfo("导航位置管理器初始化完成")
        rospy.loginfo("位置文件: %s" % self.positions_file)

    def get_current_position(self):
        """
        获取当前机器人的 2D 位置

        Returns:
            dict: {'x': float, 'y': float, 'theta': float} 或 None
        """
        try:
            # 等待 TF 变换
            self.tf_listener.waitForTransform(
                'map', 'base_footprint',
                rospy.Time(0), rospy.Duration(3.0)
            )

            # 获取位置
            (trans, rot) = self.tf_listener.lookupTransform(
                'map', 'base_footprint', rospy.Time(0)
            )

            # 将四元数转换为欧拉角
            euler = tf.transformations.euler_from_quaternion(rot)
            theta = euler[2]  # yaw 角

            position = {
                'x': trans[0],
                'y': trans[1],
                'theta': theta
            }

            return position

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("获取位置失败: %s" % e)
            return None

    def save_position(self, position_name):
        """
        保存当前 2D 位置

        Args:
            position_name (str): 位置名称

        Returns:
            bool: 是否成功
        """
        rospy.loginfo("保存位置: %s" % position_name)

        position = self.get_current_position()
        if not position:
            rospy.logerr("无法获取当前位置")
            return False

        # 读取现有位置
        all_positions = {}
        if os.path.exists(self.positions_file):
            try:
                with open(self.positions_file, 'r') as f:
                    all_positions = json.load(f)
            except Exception as e:
                rospy.logwarn("读取位置文件失败: %s" % e)

        # 添加新位置
        all_positions[position_name] = position

        # 保存到文件
        try:
            with open(self.positions_file, 'w') as f:
                json.dump(all_positions, f, indent=2)

            rospy.loginfo("✓ 位置已保存:")
            rospy.loginfo("  x: %.3f m" % position['x'])
            rospy.loginfo("  y: %.3f m" % position['y'])
            rospy.loginfo("  theta: %.3f rad (%.1f°)" % (position['theta'], position['theta'] * 180 / 3.14159))
            rospy.loginfo("  文件: %s" % self.positions_file)

            return True
        except Exception as e:
            rospy.logerr("保存位置失败: %s" % e)
            return False

    def load_position(self, position_name):
        """
        加载保存的位置

        Args:
            position_name (str): 位置名称

        Returns:
            dict: 位置信息 或 None
        """
        if not os.path.exists(self.positions_file):
            rospy.logerr("位置文件不存在: %s" % self.positions_file)
            return None

        try:
            with open(self.positions_file, 'r') as f:
                all_positions = json.load(f)

            if position_name not in all_positions:
                rospy.logerr("未找到位置: %s" % position_name)
                rospy.loginfo("可用位置: %s" % list(all_positions.keys()))
                return None

            return all_positions[position_name]

        except Exception as e:
            rospy.logerr("加载位置失败: %s" % e)
            return None

    def goto_position(self, position_name):
        """
        导航到保存的位置

        Args:
            position_name (str): 位置名称

        Returns:
            bool: 是否成功
        """
        rospy.loginfo("导航到位置: %s" % position_name)

        # 加载位置
        position = self.load_position(position_name)
        if not position:
            return False

        rospy.loginfo("目标位置:")
        rospy.loginfo("  x: %.3f m" % position['x'])
        rospy.loginfo("  y: %.3f m" % position['y'])
        rospy.loginfo("  theta: %.3f rad (%.1f°)" % (position['theta'], position['theta'] * 180 / 3.14159))

        # 创建导航目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = position['x']
        goal.target_pose.pose.position.y = position['y']
        goal.target_pose.pose.position.z = 0.0

        # 将 theta 转换为四元数
        quat = tf.transformations.quaternion_from_euler(0, 0, position['theta'])
        goal.target_pose.pose.orientation = Quaternion(*quat)

        # 发送目标
        rospy.loginfo("发送导航目标...")
        self.move_base_client.send_goal(goal)

        # 等待结果
        rospy.loginfo("等待导航完成...")
        self.move_base_client.wait_for_result()

        # 检查结果
        state = self.move_base_client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("✓ 导航成功！已到达位置: %s" % position_name)
            return True
        else:
            rospy.logerr("✗ 导航失败 (状态: %d)" % state)
            return False

    def list_positions(self):
        """列出所有保存的位置"""
        if not os.path.exists(self.positions_file):
            rospy.loginfo("还没有保存任何位置")
            return

        try:
            with open(self.positions_file, 'r') as f:
                all_positions = json.load(f)

            rospy.loginfo("已保存的位置:")
            for name, pos in all_positions.items():
                rospy.loginfo("  - %s: (x=%.2f, y=%.2f, theta=%.2f)" %
                            (name, pos['x'], pos['y'], pos['theta']))

        except Exception as e:
            rospy.logerr("读取位置文件失败: %s" % e)


def main():
    """主函数"""
    if len(sys.argv) < 2:
        print("用法:")
        print("  保存当前位置:  rosrun object_detection_world save_and_goto_nav_position.py save <位置名称>")
        print("  导航到位置:    rosrun object_detection_world save_and_goto_nav_position.py goto <位置名称>")
        print("  列出所有位置:  rosrun object_detection_world save_and_goto_nav_position.py list")
        print("")
        print("示例:")
        print("  rosrun object_detection_world save_and_goto_nav_position.py save current_pos")
        print("  rosrun object_detection_world save_and_goto_nav_position.py goto current_pos")
        return

    try:
        manager = NavPositionManager()
        rospy.sleep(0.5)  # 等待初始化

        command = sys.argv[1].lower()

        if command == 'save':
            if len(sys.argv) < 3:
                rospy.logerr("请指定位置名称")
                return
            position_name = sys.argv[2]
            manager.save_position(position_name)

        elif command == 'goto':
            if len(sys.argv) < 3:
                rospy.logerr("请指定位置名称")
                return
            position_name = sys.argv[2]
            manager.goto_position(position_name)

        elif command == 'list':
            manager.list_positions()

        else:
            rospy.logerr("未知命令: %s" % command)
            rospy.loginfo("可用命令: save, goto, list")

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
