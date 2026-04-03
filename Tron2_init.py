#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
import json
import uuid
import threading
import time
import websocket
import numpy as np

# ===============================
# 机器人配置
# ===============================
ACCID = "DACH_TRON2A_006"
ROBOT_IP = "10.192.1.2"

# ===============================
# 全局变量
# ===============================
ws_client = None
latest_cmd = None          # 最近一次 ROS 指令（list）
last_sent_joint = None     # 上一次已发送的关节
lock = threading.Lock()

# ===============================
# WebSocket 工具函数
# ===============================
def generate_guid():
    return str(uuid.uuid4())

def send_request(title, data=None):
    if data is None:
        data = {}

    if ws_client is None:
        return

    message = {
        "accid": ACCID,
        "title": title,
        "timestamp": int(time.time() * 1000),
        "guid": generate_guid(),
        "data": data
    }

    try:
        ws_client.send(json.dumps(message))
    except Exception as e:
        rospy.logwarn(f"WebSocket send failed: {e}")

# ===============================
# WebSocket 回调
# ===============================
def on_open(ws):
    rospy.loginfo("✅ WebSocket 已连接机器人")

def on_close(ws, *args):
    rospy.logwarn("❌ WebSocket 已断开")

# ===============================
# WebSocket 线程
# ===============================
def websocket_thread():
    global ws_client

    ws_client = websocket.WebSocketApp(
        f"ws://{ROBOT_IP}:5000",
        on_open=on_open,
        on_close=on_close
    )

    ws_client.run_forever()

# ===============================
# 发送关节指令（防抖）
# ===============================
def send_joint_command(joint):
    global last_sent_joint

    if last_sent_joint is not None:
        diff = np.max(np.abs(np.array(joint) - np.array(last_sent_joint)))
        if diff < 1e-4:
            return  # 没变化，不发

    send_request("request_movej", {
        "joint": joint,
        "time": 1.0
    })

    last_sent_joint = joint.copy()

# ===============================
# 发送夹爪指令
# ===============================
def send_gripper_command(grip):
    send_request("request_set_limx_2fclaw_cmd", {
        "left_opening":  grip[0],
        "left_speed":    grip[1],
        "left_force":    grip[2],
        "right_opening": grip[3],
        "right_speed":   grip[4],
        "right_force":   grip[5],
    })

# ===============================
# ROS 回调
# ===============================
def JointCallback(msg: Float32MultiArray):
    global latest_cmd

    with lock:
        # ⚠️ 关键修复：强制转 list
        latest_cmd = list(msg.data)

# ===============================
# 控制循环（定时发送）
# ===============================
def control_loop(event):
    global latest_cmd

    with lock:
        if latest_cmd is None or len(latest_cmd) < 20:
            return

        joint = latest_cmd[0:14]
        gripper = latest_cmd[14:20]

    send_joint_command(joint)
    send_gripper_command(gripper)

# ===============================
# 主程序
# ===============================
if __name__ == "__main__":
    rospy.init_node("Tron2_init")

    rospy.Subscriber(
        "/Tron2_JointCommand",
        Float32MultiArray,
        JointCallback,
        queue_size=1
    )

    rospy.Timer(rospy.Duration(0.1), control_loop)

    # WebSocket 独立线程
    threading.Thread(target=websocket_thread, daemon=True).start()

    rospy.loginfo("🚀 Tron2_init 已启动（稳定版）")
    rospy.spin()
