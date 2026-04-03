#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import os
import threading
import time
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO


class YOLOGripperDetectorFast:
    def __init__(self):
        rospy.init_node("yolo_gripper_detector_fast")

        # -----------------------------
        # 参数
        # -----------------------------
        self.conf_thres = 0.4          # 置信度阈值
        self.infer_interval = 0.2      # YOLO 推理周期（秒）≈ 5 FPS
        self.input_size = 416          # YOLO 输入尺寸

        # -----------------------------
        # YOLO 模型
        # -----------------------------
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, "best.pt")

        if not os.path.exists(model_path):
            rospy.logerr("❌ 找不到 best.pt")
            raise RuntimeError("Model not found")

        self.model = YOLO(model_path)
        rospy.loginfo("✅ YOLO 模型加载完成")

        # -----------------------------
        # ROS
        # -----------------------------
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        self.pub = rospy.Publisher(
            "/gripper_detector/center",
            Point,
            queue_size=10
        )

        # -----------------------------
        # 共享数据
        # -----------------------------
        self.latest_frame = None
        self.lock = threading.Lock()
        self.last_infer_time = 0

        # -----------------------------
        # 启动 YOLO 线程
        # -----------------------------
        self.thread = threading.Thread(target=self.yolo_loop)
        self.thread.daemon = True
        self.thread.start()

        rospy.loginfo("🚀 YOLO 夹爪检测（不卡版）已启动")

    # ==========================================================
    # ROS 图像回调（只存最新帧，绝不跑 YOLO）
    # ==========================================================
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CvBridge error: %s", e)
            return

        with self.lock:
            self.latest_frame = frame

    # ==========================================================
    # YOLO 推理线程（限频）
    # ==========================================================
    def yolo_loop(self):
        while not rospy.is_shutdown():
            now = time.time()
            if now - self.last_infer_time < self.infer_interval:
                time.sleep(0.01)
                continue

            self.last_infer_time = now

            with self.lock:
                if self.latest_frame is None:
                    continue
                frame = self.latest_frame.copy()

            self.run_yolo(frame)

    # ==========================================================
    # YOLO 推理 & 发布
    # ==========================================================
    def run_yolo(self, frame):
        h, w = frame.shape[:2]

        # resize
        frame_small = cv2.resize(frame, (self.input_size, self.input_size))

        results = self.model(
            frame_small,
            conf=self.conf_thres,
            verbose=False
        )

        boxes = results[0].boxes
        best_box = None
        best_conf = 0.0

        if boxes is not None:
            for i in range(len(boxes)):
                conf = float(boxes.conf[i])
                if conf > best_conf:
                    best_conf = conf
                    best_box = boxes.xyxy[i].cpu().numpy()

        cx, cy = -1, -1

        if best_box is not None:
            x1, y1, x2, y2 = best_box

            # 映射回原图
            scale_x = w / self.input_size
            scale_y = h / self.input_size

            x1 = int(x1 * scale_x)
            x2 = int(x2 * scale_x)
            y1 = int(y1 * scale_y)
            y2 = int(y2 * scale_y)

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # 可视化（不使用 plot！）
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(
                frame,
                f"conf:{best_conf:.2f}",
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1
            )

        # 发布坐标
        p = Point()
        p.x = float(cx)
        p.y = float(cy)
        p.z = 0.0
        self.pub.publish(p)

        # 显示
        cv2.imshow("YOLO Gripper (fast)", frame)
        cv2.waitKey(1)


if __name__ == "__main__":
    YOLOGripperDetectorFast()
    rospy.spin()
