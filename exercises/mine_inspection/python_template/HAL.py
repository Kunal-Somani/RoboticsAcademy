import rclpy
import threading
import sys

from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from hal_interfaces.general.motors import MotorsNode
from hal_interfaces.general.odometry import OdometryNode
from hal_interfaces.general.laser import LaserNode
from hal_interfaces.general.camera import CameraNode

IMG_WIDTH = 320
IMG_HEIGHT = 240


def custom_thread_excepthook(args):
    if "spin" in args.thread.name:
        return
    sys.__excepthook__(args.exc_type, args.exc_value, args.exc_traceback)


threading.excepthook = custom_thread_excepthook


class GasSensorNode(Node):
    """Subscribe to a Float32 virtual gas concentration topic."""

    def __init__(self, topic, node_name):
        super().__init__(node_name)
        self._lock = threading.Lock()
        self._value = 0.0
        self.create_subscription(Float32, topic, self._callback, 10)

    def _callback(self, msg: Float32) -> None:
        with self._lock:
            self._value = msg.data

    def getValue(self) -> float:
        with self._lock:
            return self._value


class ImuNode(Node):
    """Subscribe to /mine_rover/imu and expose latest reading."""

    def __init__(self):
        super().__init__("hal_imu_node")
        self._lock = threading.Lock()
        self._msg = None
        self.create_subscription(Imu, "/mine_rover/imu", self._callback, 10)

    def _callback(self, msg: Imu) -> None:
        with self._lock:
            self._msg = msg

    def getImuData(self):
        """Return latest IMU data as a dict, or None if not yet received.

        Keys:
            orientation         – dict(x, y, z, w)  quaternion
            angular_velocity    – dict(x, y, z)      rad/s
            linear_acceleration – dict(x, y, z)      m/s²
        """
        with self._lock:
            msg = self._msg
        if msg is None:
            return None
        return {
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w,
            },
            "angular_velocity": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z,
            },
            "linear_acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z,
            },
        }


if not rclpy.ok():
    rclpy.init(args=None)
    rclpy.create_node("HAL")

pose3d = OdometryNode("/odom")
motors = MotorsNode("/cmd_vel", 0.5, 1.0)
laser = LaserNode("/mine_rover/scan")
camera = CameraNode("/mine_rover/camera/image_raw")
imu = ImuNode()
ch4_sensor = GasSensorNode("/mine_rover/gas/ch4", "ch4_sensor_node")
co_sensor = GasSensorNode("/mine_rover/gas/co", "co_sensor_node")
crack_sensor = GasSensorNode("/mine_rover/cracks/detected", "crack_sensor_node")

executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(pose3d)
executor.add_node(laser)
executor.add_node(camera)
executor.add_node(imu)
executor.add_node(ch4_sensor)
executor.add_node(co_sensor)
executor.add_node(crack_sensor)

executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()

print("HAL-Nodes Thread Started")


def getPose3d():
    return pose3d.getPose3d()


def getLaserData():
    laser_data = laser.getLaserData()
    while len(laser_data.values) == 0:
        laser_data = laser.getLaserData()
    return laser_data


def getImage():
    image = camera.getImage()
    while image is None:
        image = camera.getImage()
    return image.data


def getIMU():
    """Return latest IMU reading as a dict, or None if not yet received.

    Keys:
        orientation         – dict(x, y, z, w)  quaternion
        angular_velocity    – dict(x, y, z)      rad/s
        linear_acceleration – dict(x, y, z)      m/s²

    Example usage (EKF-style yaw extraction)::

        data = HAL.getIMU()
        if data:
            yaw = data["orientation"]["z"]
    """
    return imu.getImuData()


def getCH4():
    """Return CH4 concentration in range [0.0, 1.0]. 1.0 = pocket centre."""
    return ch4_sensor.getValue()


def getCO():
    """Return CO concentration in range [0.0, 1.0]. 1.0 = pocket centre."""
    return co_sensor.getValue()


def getCrackDetected():
    """Return True if rover is within detection radius of a crack zone."""
    return bool(crack_sensor.getValue() > 0.5)


def setV(velocity):
    motors.sendV(float(velocity))


def setW(velocity):
    motors.sendW(float(velocity))
