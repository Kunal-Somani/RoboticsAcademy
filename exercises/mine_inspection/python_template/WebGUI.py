import base64
import json
import threading
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32

from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode
from hal_interfaces.general.camera import CameraNode
from gui_interfaces.general.measuring_threading_gui_harmonic import (
    MeasuringThreadingGUI,
)
from console_interfaces.general.console import start_console

from map import Map


class GasSensorSub(Node):
    """Subscribe to a Float32 topic and cache the latest value."""

    def __init__(self, topic, node_name):
        super().__init__(node_name)
        self.last_value_ = 0.0
        self.create_subscription(Float32, topic, self._cb, 10)

    def _cb(self, msg):
        self.last_value_ = msg.data

    def getValue(self):
        return self.last_value_


class WebGUI(MeasuringThreadingGUI):
    def __init__(self, host="ws://127.0.0.1:2303"):
        super().__init__(host)

        self.payload = {
            "camera": "",
            "map": "",
            "readings": "",
        }

        self.pose3d_node = None
        self.laser_node = None
        self.camera_node = None
        self.ch4_node = None
        self.co_node = None
        self.crack_node = None

        self.executor = None
        self.executor_thread = None

        self._setup_ros2()
        self.start()

    def _setup_ros2(self):
        if not rclpy.ok():
            rclpy.init(args=sys.argv)

        self.pose3d_node = OdometryNode("/odom")
        self.laser_node = LaserNode("/mine_rover/scan")
        self.camera_node = CameraNode("/mine_rover/camera/image_raw")
        self.ch4_node = GasSensorSub("/mine_rover/gas/ch4", "gui_ch4_sub")
        self.co_node = GasSensorSub("/mine_rover/gas/co", "gui_co_sub")
        self.crack_node = GasSensorSub("/mine_rover/cracks/detected", "gui_crack_sub")

        self.map = Map(self.get_laser_data, self.get_pose3d)

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.pose3d_node)
        self.executor.add_node(self.laser_node)
        self.executor.add_node(self.camera_node)
        self.executor.add_node(self.ch4_node)
        self.executor.add_node(self.co_node)
        self.executor.add_node(self.crack_node)

        self.executor_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
            name="webgui_ros2_executor",
        )
        self.executor_thread.start()

    def get_pose3d(self):
        if self.pose3d_node is None:
            return None
        return self.pose3d_node.getPose3d()

    def get_laser_data(self):
        if self.laser_node is None:
            return None
        laser_data = self.laser_node.getLaserData()
        if laser_data is None:
            return None
        while laser_data is not None and len(laser_data.values) == 0:
            laser_data = self.laser_node.getLaserData()
        return laser_data

    def get_camera_image(self):
        if self.camera_node is None:
            return None
        image = self.camera_node.getImage()
        if image is None:
            return None
        return image.data

    def _encode_jpeg(self, frame):
        if frame is None:
            return None
        if not isinstance(frame, np.ndarray):
            return None
        ok, buf = cv2.imencode(".jpg", frame)
        if not ok:
            return None
        return base64.b64encode(buf).decode("utf-8")

    def update_gui(self):
        camera_frame = self.get_camera_image()
        camera_b64 = self._encode_jpeg(camera_frame)

        self.map.set_readings(
            self.ch4_node.getValue(),
            self.co_node.getValue(),
            bool(self.crack_node.getValue() > 0.5),
        )
        self.map.update()
        map_frame = self.map.render()
        map_b64 = self._encode_jpeg(map_frame)

        pose = self.get_pose3d()
        pose_list = [pose.x, pose.y, pose.yaw] if pose is not None else [0, 0, 0]

        readings = {
            "ch4": self.ch4_node.getValue(),
            "co": self.co_node.getValue(),
            "crack": self.crack_node.getValue() > 0.5,
            "pose": pose_list,
        }

        self.payload["camera"] = camera_b64 if camera_b64 is not None else ""
        self.payload["map"] = map_b64 if map_b64 is not None else ""
        self.payload["readings"] = json.dumps(readings)

        message = json.dumps(self.payload)
        self.send_to_client(message)

    def reset_gui(self):
        self.map.reset()

    def __del__(self):
        try:
            if self.executor:
                self.executor.shutdown()
        except Exception:
            pass


host = "ws://127.0.0.1:2303"
gui = WebGUI(host)

start_console()
