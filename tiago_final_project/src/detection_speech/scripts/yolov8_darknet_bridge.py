#!/usr/bin/env python3
import time
import rospy
import numpy as np
import cv2
import os

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

# darknet_ros message types
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox


class YoloV8DarknetBridge:
    def __init__(self):
        rospy.init_node("yolov8_darknet_bridge", anonymous=False)
        os.environ["CUDA_VISIBLE_DEVICES"] = "" 

        self.bridge = CvBridge()

        # ROS params
        self.image_topic = rospy.get_param("~image_topic", "/xtion/rgb/image_raw")
        self.model_path  = rospy.get_param("~model", "yolov8n.pt")
        self.device      = rospy.get_param("~device", "cpu")         # 0=GPU, "cpu"=CPU
        self.conf        = float(rospy.get_param("~conf", 0.35))
        self.iou         = float(rospy.get_param("~iou", 0.5))
        self.classes     = rospy.get_param("~classes", [])       # e.g. [39] ; [] = all
        self.publish_vis = bool(rospy.get_param("~publish_vis", True))
        self.line_thick  = int(rospy.get_param("~line_thickness", 2))
        self.font_scale  = float(rospy.get_param("~font_scale", 0.6))

        # Publish topics (match darknet_ros conventions)
        self.pub_boxes = rospy.Publisher("/darknet_ros/bounding_boxes", BoundingBoxes, queue_size=10)
        self.pub_vis   = rospy.Publisher("/darknet_ros/detection_image", Image, queue_size=1) if self.publish_vis else None

        # Load YOLOv8
        self.model = YOLO(self.model_path)
        rospy.loginfo("Loaded YOLOv8 model: %s", self.model_path)
        rospy.loginfo("Subscribing image: %s", self.image_topic)
        rospy.loginfo("Publishing: /darknet_ros/bounding_boxes%s",
                      " and /darknet_ros/detection_image" if self.publish_vis else "")

        # FPS stats
        self._t_prev = time.time()
        self._fps_ema = 0.0

        rospy.Subscriber(self.image_topic, Image, self.cb, queue_size=1, buff_size=2**24)

    def _update_fps(self):
        now = time.time()
        dt = now - self._t_prev
        self._t_prev = now
        if dt > 0:
            inst = 1.0 / dt
            self._fps_ema = 0.9 * self._fps_ema + 0.1 * inst
        return self._fps_ema

    def cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge error: %s", str(e))
            return

        h, w = frame.shape[:2]
        vis = frame.copy() if (self.publish_vis and self.pub_vis is not None) else None

        # Run YOLOv8 inference
        try:
            results = self.model.predict(
                source=frame,
                conf=self.conf,
                iou=self.iou,
                classes=self.classes if len(self.classes) > 0 else None,
                device="cpu",
                verbose=False
            )
        except Exception as e:
            rospy.logerr("YOLOv8 predict failed: %s", str(e))
            return

        r0 = results[0]
        names = getattr(r0, "names", {})  # dict: id->name
        rospy.loginfo("Detections: %d", len(r0.boxes) if r0.boxes is not None else 0)   

        # Assemble BoundingBoxes message
        out = BoundingBoxes()
        out.header = msg.header
        out.image_header = msg.header

        if r0.boxes is not None and len(r0.boxes) > 0:
            for b in r0.boxes:
                # xyxy float
                x1, y1, x2, y2 = b.xyxy[0].detach().cpu().numpy().tolist()

                xmin = int(np.clip(round(x1), 0, w - 1))
                ymin = int(np.clip(round(y1), 0, h - 1))
                xmax = int(np.clip(round(x2), 0, w - 1))
                ymax = int(np.clip(round(y2), 0, h - 1))

                # filter invalid
                if xmax <= xmin or ymax <= ymin:
                    continue

                cls_id = int(b.cls[0].item())
                prob   = float(b.conf[0].item())
                cls_name = str(names.get(cls_id, cls_id))

                bb = BoundingBox()
                bb.xmin = xmin
                bb.ymin = ymin
                bb.xmax = xmax
                bb.ymax = ymax
                bb.id = cls_id
                bb.Class = cls_name
                bb.probability = prob
                out.bounding_boxes.append(bb)

                # Visualization
                if vis is not None:
                    cv2.rectangle(vis, (xmin, ymin), (xmax, ymax), (0, 255, 0), self.line_thick)
                    label = f"{cls_name} {prob:.2f}"
                    y_text = max(0, ymin - 6)
                    cv2.putText(vis, label, (xmin, y_text),
                                cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, (0, 255, 0), 2)

        # Publish bbox
        self.pub_boxes.publish(out)

        # Publish visualization image
        if vis is not None:
            fps = self._update_fps()
            cv2.putText(vis, f"FPS: {fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            try:
                vis_msg = self.bridge.cv2_to_imgmsg(vis, "bgr8")
                vis_msg.header = msg.header
                self.pub_vis.publish(vis_msg)
            except Exception as e:
                rospy.logwarn("Publish detection_image failed: %s", str(e))


if __name__ == "__main__":
    YoloV8DarknetBridge()
    rospy.spin()
