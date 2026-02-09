#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TTS Server Node for TIAGo

Responsibilities:
- Subscribe to /tts_request (std_msgs/String)
- Forward requests to TIAGo /tts action server
- Handle cooldown to avoid repeated speech

This node decouples speech from state machines and perception modules.

Author: Your Name
"""

import time
import rospy
import actionlib
from std_msgs.msg import String
from pal_interaction_msgs.msg import TtsAction, TtsGoal


class TTSServerNode:
    def __init__(self):
        rospy.init_node("tts_server_node", anonymous=False)

        # ======================================================
        # Parameters
        # ======================================================
        self.tts_action_name = rospy.get_param("~tts_action", "/tts")
        self.language = rospy.get_param("~language", "en_GB")
        self.cooldown_sec = float(rospy.get_param("~cooldown_sec", 3.0))
        self.wait_before_speaking = float(rospy.get_param("~wait_before_speaking", 0.0))

        # ======================================================
        # Internal state
        # ======================================================
        self._last_speak_time = 0.0

        # ======================================================
        # Action client
        # ======================================================
        rospy.loginfo("Connecting to TTS action server: %s", self.tts_action_name)
        self.client = actionlib.SimpleActionClient(self.tts_action_name, TtsAction)

        if not self.client.wait_for_server(rospy.Duration(20.0)):
            rospy.logerr("TTS action server not available. Node will still run, but speech will fail.")
        else:
            rospy.loginfo("Connected to TTS action server.")

        # ======================================================
        # Subscriber
        # ======================================================
        self.sub = rospy.Subscriber(
            "/tts_request",
            String,
            self._cb_tts_request,
            queue_size=10
        )

        rospy.loginfo("TTSServerNode started. Listening on /tts_request")

    # ======================================================
    # Callback
    # ======================================================
    def _cb_tts_request(self, msg: String):
        text = msg.data.strip()
        if not text:
            rospy.logwarn("Received empty TTS request, ignoring.")
            return

        now = time.time()
        if (now - self._last_speak_time) < self.cooldown_sec:
            rospy.logwarn_throttle(
                1.0,
                "TTS cooldown active (%.2fs), skipping: '%s'",
                self.cooldown_sec,
                text
            )
            return

        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = self.language
        goal.speakerName = ""
        goal.wait_before_speaking = self.wait_before_speaking

        try:
            self.client.send_goal(goal)
            self._last_speak_time = now
            rospy.loginfo("TTS speaking: \"%s\"", text)
        except Exception as e:
            rospy.logerr("Failed to send TTS goal: %s", str(e))


if __name__ == "__main__":
    TTSServerNode()
    rospy.spin()
