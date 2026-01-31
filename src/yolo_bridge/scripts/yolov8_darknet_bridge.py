#!/usr/bin/env python3
import time
import rospy
import numpy as np
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

# darknet_ros message types
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox


class YoloV8DarknetBridge:
    def __init__(self):
        rospy.init_node("yolov8_darknet_bridge", anonymous=False)

        self.bridge = CvBridge()

        # ROS 参数：订阅图像、模型与推理配置
        self.image_topic = rospy.get_param("~image_topic", "/xtion/rgb/image_raw")
        self.model_path  = rospy.get_param("~model", "yolov8n.pt")
        self.device      = rospy.get_param("~device", 0)         # 0=GPU, "cpu"=CPU
        self.conf        = float(rospy.get_param("~conf", 0.24))
        self.iou         = float(rospy.get_param("~iou", 0.5))
        self.classes     = rospy.get_param("~classes", [])       # 例如 [39]；空列表表示全部类别
        self.publish_vis = bool(rospy.get_param("~publish_vis", True))
        self.line_thick  = int(rospy.get_param("~line_thickness", 2))
        self.font_scale  = float(rospy.get_param("~font_scale", 0.6))

        # 发布话题（对齐 darknet_ros 的消息命名）
        self.pub_boxes = rospy.Publisher("/darknet_ros/bounding_boxes", BoundingBoxes, queue_size=10)
        self.pub_vis   = rospy.Publisher("/darknet_ros/detection_image", Image, queue_size=1) if self.publish_vis else None

        # 加载 YOLOv8 模型
        self.model = YOLO(self.model_path)
        rospy.loginfo("Loaded YOLOv8 model: %s", self.model_path)
        rospy.loginfo("Subscribing image: %s", self.image_topic)
        rospy.loginfo("Publishing: /darknet_ros/bounding_boxes%s",
                      " and /darknet_ros/detection_image" if self.publish_vis else "")

        # FPS 统计（指数滑动平均）
        self._t_prev = time.time()
        self._fps_ema = 0.0

        # 订阅图像话题
        rospy.Subscriber(self.image_topic, Image, self.cb, queue_size=1, buff_size=2**24)

    def _update_fps(self):
        # 计算 FPS（使用 EMA 平滑）
        now = time.time()
        dt = now - self._t_prev
        self._t_prev = now
        if dt > 0:
            inst = 1.0 / dt
            self._fps_ema = 0.9 * self._fps_ema + 0.1 * inst
        return self._fps_ema

    def cb(self, msg: Image):
        # ROS Image -> OpenCV 图像
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("cv_bridge error: %s", str(e))
            return

        h, w = frame.shape[:2]
        # 可视化图像副本（可选）
        vis = frame.copy() if (self.publish_vis and self.pub_vis is not None) else None

        # 运行 YOLOv8 推理
        try:
            results = self.model.predict(
                source=frame,
                conf=self.conf,
                iou=self.iou,
                classes=self.classes if len(self.classes) > 0 else None,
                device=self.device,
                verbose=False
            )
        except Exception as e:
            rospy.logerr("YOLOv8 predict failed: %s", str(e))
            return

        r0 = results[0]
        names = getattr(r0, "names", {})  # 字典：类别 id -> 名称
        rospy.loginfo("Detections: %d", len(r0.boxes) if r0.boxes is not None else 0)   

        # 组装 BoundingBoxes 消息
        out = BoundingBoxes()
        out.header = msg.header
        out.image_header = msg.header

        if r0.boxes is not None and len(r0.boxes) > 0:
            for b in r0.boxes:
                # 取出检测框坐标（xyxy，浮点）
                x1, y1, x2, y2 = b.xyxy[0].detach().cpu().numpy().tolist()

                # 裁剪并转换为像素坐标
                xmin = int(np.clip(round(x1), 0, w - 1))
                ymin = int(np.clip(round(y1), 0, h - 1))
                xmax = int(np.clip(round(x2), 0, w - 1))
                ymax = int(np.clip(round(y2), 0, h - 1))

                # 过滤非法框
                if xmax <= xmin or ymax <= ymin:
                    continue

                cls_id = int(b.cls[0].item())
                prob   = float(b.conf[0].item())
                cls_name = str(names.get(cls_id, cls_id))

                # 填充 darknet_ros 的 BoundingBox 字段
                bb = BoundingBox()
                bb.xmin = xmin
                bb.ymin = ymin
                bb.xmax = xmax
                bb.ymax = ymax
                bb.id = cls_id
                bb.Class = cls_name
                bb.probability = prob
                out.bounding_boxes.append(bb)

                # 可视化绘制
                if vis is not None:
                    cv2.rectangle(vis, (xmin, ymin), (xmax, ymax), (0, 255, 0), self.line_thick)
                    label = f"{cls_name} {prob:.2f}"
                    y_text = max(0, ymin - 6)
                    cv2.putText(vis, label, (xmin, y_text),
                                cv2.FONT_HERSHEY_SIMPLEX, self.font_scale, (0, 255, 0), 2)

        # 发布检测框
        self.pub_boxes.publish(out)

        # 发布可视化图像
        if vis is not None:
            fps = self._update_fps()
            cv2.putText(vis, f"FPS: {fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            try:
                # OpenCV 图像 -> ROS Image 并发布
                vis_msg = self.bridge.cv2_to_imgmsg(vis, "bgr8")
                vis_msg.header = msg.header
                self.pub_vis.publish(vis_msg)
            except Exception as e:
                rospy.logwarn("Publish detection_image failed: %s", str(e))


if __name__ == "__main__":
    YoloV8DarknetBridge()
    rospy.spin()
