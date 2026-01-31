#!/usr/bin/env python3
import rospy
import actionlib
from std_msgs.msg import String
from pal_interaction_msgs.msg import TtsAction, TtsGoal

class GuestTTSBridge:
    def __init__(self):
        rospy.init_node("guest_tts_bridge", anonymous=False)

        self.lang = rospy.get_param("~lang", "en_GB")  # 语言
        self.cooldown_sec = float(rospy.get_param("~cooldown_sec", 3.0))  # 防止短时间重复说话
        self.last_spoke = 0.0

        self.client = actionlib.SimpleActionClient("/tts", TtsAction)
        rospy.loginfo("Waiting for /tts action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to /tts.")

        rospy.Subscriber("/guest_event", String, self.cb, queue_size=10)

    def say(self, text):
        now = rospy.get_time()
        if (now - self.last_spoke) < self.cooldown_sec:
            return
        self.last_spoke = now

        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = self.lang
        self.client.send_goal(goal)

    def cb(self, msg: String):
        s = msg.data.strip()

        # 你 yolo 节点发的是 guest_wave_detected:<count>
        if s.startswith("guest_wave_detected:"):
            try:
                n = int(s.split(":")[-1])
            except Exception:
                n = 1
            if n <= 1:
                self.say("Hello, welcome!")
            else:
                self.say(f"Hello, welcome! I see {n} guests.")
        elif s in ("session_end", "session_timeout_end"):
            # 可选：结束时说一句
            pass

if __name__ == "__main__":
    GuestTTSBridge()
    rospy.spin()
