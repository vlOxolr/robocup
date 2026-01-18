#!/usr/bin/env python3
import time
import threading
from collections import deque

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from ultralytics import YOLO
import actionlib
from pal_interaction_msgs.msg import TtsAction, TtsGoal


class WaveGreeterImshow:
    def __init__(self):
        rospy.init_node("yolo_wave_greeter_imshow_ros", anonymous=False)

        # --- Params ---
        self.image_topic = rospy.get_param("~image_topic", "/xtion/rgb/image_raw")
        self.det_model_path = rospy.get_param("~det_model", "yolov8n.pt")
        self.pose_model_path = rospy.get_param("~pose_model", "yolov8n-pose.pt")
        self.device = rospy.get_param("~device", 0)  # 0=GPU, "cpu"=CPU

        self.conf_det = float(rospy.get_param("~conf_det", 0.35))  # 目标检测置信度阈值(人框)
        self.iou_det = float(rospy.get_param("~iou_det", 0.5))
        self.conf_pose = float(rospy.get_param("~conf_pose", 0.45))  # 姿态置信度阈值(建议提高降低误触发)
        self.iou_pose = float(rospy.get_param("~iou_pose", 0.5))

        self.process_hz = float(rospy.get_param("~process_hz", 8.0))  # 处理频率(Hz)，越高越吃GPU

        # ====== 迎宾流程相关 ======
        self.empty_reset_sec = float(rospy.get_param("~empty_reset_sec", 2.0))  # 画面无人持续多久 -> 结束本轮
        self.session_timeout_sec = float(rospy.get_param("~session_timeout_sec", 30.0))  # 本轮最长时长(兜底)，0=关闭

        # ====== 挥手识别(左右挥)相关：重点调这些 ======
        self.wave_time_sec = float(rospy.get_param("~wave_time_sec", 1.0))
        # 中文：在多少秒的时间窗内判断“左右挥动”。太小容易漏检；太大容易误触发。
        # 建议：0.8~1.5

        self.wave_min_swing_px = float(rospy.get_param("~wave_min_swing_px", 65.0))
        # 中文：手腕在X方向“左右摆动”的最小幅度(像素)。
        # 数值越大越严格(更不容易误触发)；太大可能离镜头远时触发不了。
        # 建议：近距离 80~120，远距离 50~90

        self.wave_min_dir_changes = int(rospy.get_param("~wave_min_dir_changes", 2))
        # 中文：要求手腕X方向速度“方向改变”的次数。
        # 2 表示至少出现 左->右->左 或 右->左->右 这种“来回摆动”。
        # 越大越严格，误触发更低但更难触发。
        # 建议：2 或 3

        self.wave_min_raised_frames = int(rospy.get_param("~wave_min_raised_frames", 3))
        # 中文：先要求“抬手到头附近”连续满足多少帧，才开始统计左右挥动。
        # 越大越不容易误触发，但会变“反应慢”。
        # 建议：3~6
        self.wave_raise_margin_ratio = float(rospy.get_param("~wave_raise_margin_ratio", 0.12))
        # 中文：抬手判定的裕量(相对人框高度比例)。用于判断“手腕是否高过鼻子(头部附近)”
        # 越大越严格（必须抬得更高），越小越容易触发。
        # 建议：0.12~0.25

        # --- imshow options ---
        self.show_window = bool(rospy.get_param("~show_window", True))
        self.window_name = rospy.get_param("~window_name", "YOLO WaveGreeter (annotated)")
        self.exit_on_q = bool(rospy.get_param("~exit_on_q", False))
        self.window_w = int(rospy.get_param("~window_w", 960))
        self.window_h = int(rospy.get_param("~window_h", 540))

        # --- ROS pubs/subs ---
        self.bridge = CvBridge()
        self.count_pub = rospy.Publisher("/guest_visible_count", Int32, queue_size=10)
        self.locked_pub = rospy.Publisher("/guest_locked_count", Int32, queue_size=10)
        self.event_pub = rospy.Publisher("/guest_event", String, queue_size=10)
        self.debug_pub = rospy.Publisher("/guest_counter/debug_image", Image, queue_size=1)
                # --- TTS (TIAGo) ---
        self.enable_tts = bool(rospy.get_param("~enable_tts", True))   # 是否启用说话
        self.tts_lang = rospy.get_param("~tts_lang", "en_GB")          # 语言
        self.tts_cooldown_sec = float(rospy.get_param("~tts_cooldown_sec", 5.0))  # 说话冷却，避免连续触发
        self._last_tts_time = 0.0

        self.tts_client = None
        if self.enable_tts:
            try:
                self.tts_client = actionlib.SimpleActionClient("/tts", TtsAction)
                rospy.loginfo("Waiting for /tts action server...")
                self.tts_client.wait_for_server(rospy.Duration(5.0))
                rospy.loginfo("/tts action server connected.")
            except Exception as e:
                rospy.logwarn("TTS init failed: %s", str(e))
                self.tts_client = None


        # --- Models ---
        self.det_model = YOLO(self.det_model_path)
        self.pose_model = YOLO(self.pose_model_path)

        # --- State ---
        self.state = "WAIT_WAVE"
        self.locked_count = 0
        self.lock_time = 0.0

        self.last_process_time = 0.0
        self.last_nonzero_time = time.time()

        # ====== 挥手轨迹缓存 ======
        self._wrist_hist = deque(maxlen=60)   # (t, x) 最近的手腕x轨迹点
        self._raised_hist = deque(maxlen=20)  # 最近若干帧是否“抬手到头附近”

        # latest visualization for imshow
        self._vis_lock = threading.Lock()
        self._latest_vis = None
        self._latest_vis_time = 0.0

        # create window
        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, self.window_w, self.window_h)

        # subscribe
        rospy.Subscriber(self.image_topic, Image, self.cb, queue_size=1, buff_size=2**24)
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("WaveGreeterImshow started.")
        rospy.loginfo("topic=%s det_model=%s pose_model=%s device=%s",
                      self.image_topic, self.det_model_path, self.pose_model_path, str(self.device))

    def _on_shutdown(self):
        if self.show_window:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

    def _throttle(self):
        if self.process_hz <= 0:
            return False
        now = time.time()
        min_dt = 1.0 / self.process_hz
        if (now - self.last_process_time) < min_dt:
            return True
        self.last_process_time = now
        return False

    def _count_persons(self, frame):
        results = self.det_model.predict(
            source=frame,
            conf=self.conf_det,
            iou=self.iou_det,
            classes=[0],
            device=self.device,
            verbose=False
        )
        r0 = results[0]
        count = len(r0.boxes) if r0.boxes is not None else 0
        return count, r0

    def _wave_detected(self, frame):
        """
        左右挥手检测（更稳，比“举手”误触发低很多）：
        1) 先判定“抬手到头附近”：手腕 y < 鼻子 y（带裕量）
        2) 只有持续抬手 wave_min_raised_frames 帧，才开始记录手腕 x 轨迹
        3) 在 wave_time_sec 秒内，若 x摆动幅度 >= wave_min_swing_px 且方向改变次数 >= wave_min_dir_changes -> 认为挥手
        """
        results = self.pose_model.predict(
            source=frame,
            conf=self.conf_pose,
            iou=self.iou_pose,
            classes=[0],
            device=self.device,
            verbose=False
        )
        r0 = results[0]
        kpts = getattr(r0, "keypoints", None)
        if kpts is None or kpts.xy is None or len(kpts.xy) == 0:
            self._raised_hist.append(False)
            return False, r0

        now = time.time()

        # 选出“最像在抬手挥”的那个候选（所有人中手抬得最高的那只手）
        best = None  # (wrist_x, wrist_y)
        for i in range(len(kpts.xy)):
            pts = kpts.xy[i]
            if pts is None or len(pts) < 11:
                continue

            nose = pts[0]    # 0 nose
            lw = pts[9]      # 9 left_wrist
            rw = pts[10]     # 10 right_wrist

            # margin based on bbox height (让阈值和人物大小自适应)
            margin = 20.0
            if r0.boxes is not None and len(r0.boxes) > i:
                x1, y1, x2, y2 = r0.boxes[i].xyxy[0].tolist()
                h = max(1.0, (y2 - y1))
                margin = self.wave_raise_margin_ratio * h

            # 抬手到头附近：手腕需要高过鼻子（更像招手）
            left_raised = (lw[1] + margin) < nose[1]
            right_raised = (rw[1] + margin) < nose[1]

            if left_raised or right_raised:
                cand = lw if left_raised else rw
                if right_raised and (rw[1] < lw[1]):
                    cand = rw

                if best is None or float(cand[1]) < best[1]:
                    best = (float(cand[0]), float(cand[1]))

        raised_now = best is not None
        self._raised_hist.append(raised_now)

        # 必须连续抬手若干帧才进入“挥手统计”，避免误触发
        if sum(1 for x in self._raised_hist if x) < self.wave_min_raised_frames:
            if not raised_now:
                self._wrist_hist.clear()
            return False, r0

        if not raised_now:
            self._wrist_hist.clear()
            return False, r0

        wrist_x = best[0]
        self._wrist_hist.append((now, wrist_x))

        # 只保留 wave_time_sec 秒内的轨迹
        while self._wrist_hist and (now - self._wrist_hist[0][0]) > self.wave_time_sec:
            self._wrist_hist.popleft()

        if len(self._wrist_hist) < 6:
            return False, r0

        xs = [p[1] for p in self._wrist_hist]
        swing = max(xs) - min(xs)

        # 幅度不够就不算挥手（过滤小抖动）
        if swing < self.wave_min_swing_px:
            return False, r0

        # 方向改变次数：统计 dx 的符号翻转（忽略小抖动）
        dir_changes = 0
        prev_dx = 0.0
        for j in range(1, len(xs)):
            dx = xs[j] - xs[j - 1]
            if abs(dx) < 2.0:
                continue
            if prev_dx == 0.0:
                prev_dx = dx
                continue
            if (dx > 0 and prev_dx < 0) or (dx < 0 and prev_dx > 0):
                dir_changes += 1
                prev_dx = dx

        if dir_changes >= self.wave_min_dir_changes:
            # 触发后清空轨迹，避免连续重复触发
            self._wrist_hist.clear()
            self._raised_hist.clear()
            return True, r0

        return False, r0

    def _reset_session(self, reason="session_end"):
        self.state = "WAIT_WAVE"
        self.locked_count = 0
        self.lock_time = 0.0

        # 清掉挥手历史，避免重置后立刻误触发
        self._wrist_hist.clear()
        self._raised_hist.clear()

        self.event_pub.publish(String(reason))
        rospy.loginfo("Session reset -> WAIT_WAVE (%s)", reason)
    
    def _say(self, text: str):
        """调用 TIAGo /tts action 说话（只说一次，带冷却）"""
        if not self.enable_tts or self.tts_client is None:
            return

        now = time.time()
        if (now - self._last_tts_time) < self.tts_cooldown_sec:
            return

        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = self.tts_lang
        goal.speakerName = ""
        goal.wait_before_speaking = 0.0

        try:
            self.tts_client.send_goal(goal)
            self._last_tts_time = now
        except Exception as e:
            rospy.logwarn("TTS send_goal failed: %s", str(e))


    def _annotate(self, det_r0, visible_count, pose_r0=None, debug_wave=None):
        img = det_r0.plot()
        if pose_r0 is not None:
            try:
                img = pose_r0.plot(img=img)
            except Exception:
                pass

        cv2.putText(img, f"state={self.state}", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        cv2.putText(img, f"visible={visible_count}", (20, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

        if self.state == "LOCKED":
            cv2.putText(img, f"LOCKED={self.locked_count}", (20, 105),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        else:
            # 挥手调试信息，调参
            if debug_wave is not None:
                swing, dir_changes, raised_cnt = debug_wave
                cv2.putText(img, f"swing={swing:.1f}px  dir={dir_changes}  raised={raised_cnt}",
                            (20, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        return img

    def cb(self, msg):
        if self._throttle():
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge error: %s", str(e))
            return

        now = time.time()

        visible_count, det_r0 = self._count_persons(frame)
        self.count_pub.publish(Int32(visible_count))
        if visible_count > 0:
            self.last_nonzero_time = now

        wave_now = False
        pose_r0 = None

        # 用于在画面上显示调试数值
        debug_wave = None

        if self.state == "WAIT_WAVE":
            if visible_count > 0:
                wave_now, pose_r0 = self._wave_detected(frame)

                # 计算调试信息：当前轨迹幅度/方向变化/抬手帧数
                if len(self._wrist_hist) >= 2:
                    xs = [p[1] for p in self._wrist_hist]
                    swing = max(xs) - min(xs)
                    # 方向变化（同 _wave_detected 的计算方式）
                    dir_changes = 0
                    prev_dx = 0.0
                    for j in range(1, len(xs)):
                        dx = xs[j] - xs[j - 1]
                        if abs(dx) < 2.0:
                            continue
                        if prev_dx == 0.0:
                            prev_dx = dx
                            continue
                        if (dx > 0 and prev_dx < 0) or (dx < 0 and prev_dx > 0):
                            dir_changes += 1
                            prev_dx = dx
                    raised_cnt = sum(1 for x in self._raised_hist if x)
                    debug_wave = (swing, dir_changes, raised_cnt)

            # 触发：只要任意一个人挥手成功，就锁定当前画面人数
            if wave_now and visible_count > 0:
                self.state = "LOCKED"
                self.locked_count = visible_count
                self.lock_time = now
                self.event_pub.publish(String(f"guest_wave_detected:{self.locked_count}"))
                rospy.loginfo("Wave trigger -> LOCKED, locked_count=%d", self.locked_count)
                if self.locked_count == 1:
                    self._say(f"Hello, welcome to our restaurant! I see {self.locked_count} guest. Please follow me.")
                else:
                    self._say(f"Hello, welcome to our restaurant! I see {self.locked_count} guests. Please follow me.")


        elif self.state == "LOCKED":
            self.locked_pub.publish(Int32(self.locked_count))
            if (now - self.last_nonzero_time) > self.empty_reset_sec:
                self._reset_session("session_end")
            elif self.session_timeout_sec > 0 and (now - self.lock_time) > self.session_timeout_sec:
                self._reset_session("session_timeout_end")

        annotated = self._annotate(det_r0, visible_count, pose_r0=pose_r0, debug_wave=debug_wave)

        # publish debug image
        try:
            out = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
            out.header = msg.header
            self.debug_pub.publish(out)
        except Exception:
            pass

        # store for main-thread display
        with self._vis_lock:
            self._latest_vis = annotated
            self._latest_vis_time = now

    def run_display_loop(self):
        """Main thread: keep OpenCV window responsive and updating."""
        if not self.show_window:
            rospy.spin()
            return

        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            with self._vis_lock:
                img = None if self._latest_vis is None else self._latest_vis.copy()
                ts = self._latest_vis_time

            if img is not None:
                cv2.imshow(self.window_name, img)
            else:
                blank = 255 * (cv2.UMat(480, 640, cv2.CV_8UC3)).get()
                cv2.putText(blank, "Waiting for camera frames...", (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 2)
                cv2.imshow(self.window_name, blank)

            k = cv2.waitKey(1) & 0xFF
            if self.exit_on_q and k == ord('q'):
                rospy.signal_shutdown("User pressed q")

            if img is not None and (time.time() - ts) > 2.0:
                rospy.logwarn_throttle(2.0, "Window not getting fresh frames? Check /usb_cam/image_raw hz and ROS master.")

            rate.sleep()


if __name__ == "__main__":
    node = WaveGreeterImshow()
    node.run_display_loop()