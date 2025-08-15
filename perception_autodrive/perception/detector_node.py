import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import json
import cv2

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        qos_profile = QoSProfile( # Ouality of Service profile
        reliability=QoSReliabilityPolicy.RELIABLE, # Reliable (not best effort) communication
        history=QoSHistoryPolicy.KEEP_LAST, # Keep/store only up to last N samples
        depth=1 # Queue (buffer) size/depth (only honored if the “history” policy was set to “keep last”)
        )
        self.sub = self.create_subscription(Image, '/autodrive/f1tenth_1/front_camera', self.image_callback, qos_profile)
        self.pub = self.create_publisher(String, '/perception/objects', qos_profile)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # or yolov5n.pt
        # self.model = YOLO('yolov8l.pt')

    def image_callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_img)
        output = []
        for r in results:
            for box in r.boxes:
                cls = int(box.cls.item())
                name = self.model.names[cls]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                output.append({
                    'id': id(box),
                    'class': name,
                    'bbox': [x1, y1, x2, y2]
                })
        self.pub.publish(String(data=json.dumps(output)))

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
