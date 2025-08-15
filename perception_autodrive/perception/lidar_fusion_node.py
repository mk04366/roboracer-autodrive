import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import json

class LidarFusionNode(Node):
    def __init__(self):
        super().__init__('lidar_fusion_node')

        # Declare parameters: image width and camera field of view
        self.declare_parameter('image_width', 640)
        self.declare_parameter('camera_fov_deg', 90.0)

        self.image_width = self.get_parameter('image_width').value
        self.camera_fov_deg = self.get_parameter('camera_fov_deg').value

        self.lidar_data = None

        # Set QoS profile
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1   # 10
        )

        # Subscribe to LaserScan and detected objects
        self.create_subscription(LaserScan, '/autodrive/f1tenth_1/lidar', self.lidar_callback, qos)
        self.create_subscription(String, '/perception/objects', self.objects_callback, qos)

        # Publisher for fused output
        self.pub = self.create_publisher(String, '/perception/fused_objects', qos)

    def lidar_callback(self, msg):
        # Store the latest Lidar data
        self.lidar_data = msg

    def objects_callback(self, msg):
        # Skip if no lidar data is available yet
        if self.lidar_data is None:
            self.get_logger().warn("Waiting for lidar data...")
            return

        # Parse object detection results
        try:
            objects = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Invalid JSON: {e}")
            return

        fused = []
        for obj in objects:
            try:
                bbox = obj["bbox"]  # [x1, y1, x2, y2]
                x_center = (bbox[0] + bbox[2]) / 2

                # Convert pixel position to angle relative to camera FOV
                angle_deg = ((x_center / self.image_width) - 0.5) * self.camera_fov_deg
                angle_rad = math.radians(angle_deg)

                # Offset camera-relative angle to Lidar-relative angle
                laser_angle = angle_rad + math.radians(135)
                index = int(laser_angle / self.lidar_data.angle_increment)

                # Clamp index to valid range
                index = max(0, min(index, len(self.lidar_data.ranges) - 1))

                # Read distance from Lidar ranges
                distance = self.lidar_data.ranges[index]
                if math.isinf(distance) or math.isnan(distance):
                    distance = -1.0  # Mark as invalid

                obj["distance"] = round(distance, 2)
                fused.append(obj)
            except Exception as e:
                self.get_logger().warn(f"Error processing object: {e}")
                continue

        # Publish fused results
        self.pub.publish(String(data=json.dumps(fused)))

def main(args=None):
    rclpy.init(args=args)
    node = LidarFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()