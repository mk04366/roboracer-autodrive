import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan, Imu, Image  # Import message classes for sensor data
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Ouality of Service (tune communication between nodes)
from rclpy.logging import get_logger
from attrdict import AttrDict # Mapping objects that allow their elements to be accessed both as keys and as attributes
from cv_bridge import CvBridge  # For converting ROS Image messages to OpenCV images
import cv2
import numpy as np  # For numerical operations


cv_bridge = CvBridge()  # Initialize CvBridge for image conversion

sub_dict = AttrDict({
    'subscribers': [
        {'topic': '/autodrive/f1tenth_1/lidar', 'type': LaserScan, 'name': 'sub_lidar'},
        {'topic': '/autodrive/f1tenth_1/imu', 'type': Imu, 'name': 'sub_imu'},
        {'topic': '/autodrive/f1tenth_1/front_camera', 'type': Image, 'name': 'sub_front_camera'},
    ]
})


def callback_lidar_scan(msg):
        # print('Lidar data received.')
        # # 雷达距离数组
        # ranges = np.array(msg.ranges)

        # # 清洗无效数据（小于最小范围或大于最大范围）
        # valid_ranges = np.where(
        #     (ranges > msg.range_min) & (ranges < msg.range_max),
        #     ranges,
        #     np.inf
        # )

        # # 最近障碍物距离
        # min_distance = np.min(valid_ranges)

        # # 最远距离点对应的角度
        # max_index = np.argmax(valid_ranges)
        # angle = msg.angle_min + max_index * msg.angle_increment

        # # 输出（后期可发布或写入 shared state）
        # # self.get_logger().info(f'Obstacle ahead: {min_distance:.2f} m | Free direction: {angle:.2f} rad')
        # print(f'Obstacle ahead: {min_distance:.2f} m | Free direction: {angle:.2f} rad')

        # # 可以结构化保存
        # perception_output = {
        #     'min_obstacle_distance': float(min_distance),
        #     'free_direction': float(angle)
        # }
        return

def callback_imu(msg):
    # print('IMU data received.')
    # print(f'Orientation: {msg.orientation}')
    return

def callback_front_camera(msg):
        # Convert ROS Image to OpenCV format (BGR)
    frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    height, width, _ = frame.shape
    roi = frame[int(height * 0.5):, :]  # Crop lower half for road area only

    # Create a mask to exclude vehicle parts (center-bottom rectangle)
    mask = np.ones_like(roi[:, :, 0], dtype=np.uint8) * 255  # white mask
    center_x = width // 2
    car_mask_width = int(width * 0.25)
    car_mask_height = int(height * 0.25)

    # Draw black rectangle to ignore vehicle body
    cv2.rectangle(mask,
                  (center_x - car_mask_width // 2, roi.shape[0] - car_mask_height),
                  (center_x + car_mask_width // 2, roi.shape[0]),
                  0, -1)

    # Grayscale and blur
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)

    # Apply mask to remove car area
    masked_edges = cv2.bitwise_and(edges, edges, mask=mask)

    # Hough Transform to detect lines
    lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, threshold=50, minLineLength=40, maxLineGap=150)

    mid_xs = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(roi, (x1, y1), (x2, y2), (0, 255, 0), 2)
            mid_xs.append((x1 + x2) / 2)

        avg_x = np.mean(mid_xs)
        lane_offset = (avg_x - width / 2) / (width / 2)  # Normalized offset in [-1, 1]

        print(f"[Perception] Lane offset: {lane_offset:.2f}")
    else:
        lane_offset = 0.0
        print("[Perception] No lane lines detected.")

    # Optional: Visualize result
    cv2.imwrite('detected_lane.jpg', roi)
    cv2.imshow("Lane Detection (Masked)", roi)
    cv2.waitKey(1)
    # try:
    #     print('Image data received.')
    #     cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    #     # cv2.imwrite('front_camera_image.jpg', cv_image)  # Save image for debugging
    #     cv2.imshow("Camera View", cv_image)
    #     cv2.waitKey(1)  
    # except Exception as e:
    #     print(f"[ERROR] Failed to convert image: {e}")
    return

def main(args=None):
    rclpy.init(args=args)
    read_sensor = rclpy.create_node('read_sensor')
    qos_profile = QoSProfile( # Ouality of Service profile
        reliability=QoSReliabilityPolicy.RELIABLE, # Reliable (not best effort) communication
        history=QoSHistoryPolicy.KEEP_LAST, # Keep/store only up to last N samples
        depth=1 # Queue (buffer) size/depth (only honored if the “history” policy was set to “keep last”)
        )
    callbacks = {
        '/autodrive/f1tenth_1/lidar': callback_lidar_scan,
        '/autodrive/f1tenth_1/imu': callback_imu,
        '/autodrive/f1tenth_1/front_camera': callback_front_camera,
    }  # Subscriber callback functions
    subscribers = [read_sensor.create_subscription(e.type, e.topic, callbacks[e.topic], qos_profile) for e in sub_dict.subscribers]  # Subscribers
    subscribers  # Avoid unused variable warning

    # Create a logger
    logger = get_logger('read_sensor')
    # logger.info('Lidar listener node has been started.')
    # Create and write data to shared config file
    # package_share_directory = get_package_share_directory('autodrive_f1tenth')
    # api_config = configparser.ConfigParser()
    # api_config['f1tenth_1'] = {'throttle_command': str(throttle_command),
    #                            'steering_command': str(steering_command)
    #                            }
    # with open(package_share_directory+'/api_config.ini', 'w') as configfile:
    #     api_config.write(configfile)
    
    rclpy.spin(read_sensor)
    read_sensor.destroy_node()
    rclpy.shutdown()

# class LidarPerceptionNode(Node):
#     def __init__(self):
#         super().__init__('lidar_perception_node')
        
#         qos_profile = QoSProfile(
#             reliability=QoSReliabilityPolicy.RELIABLE,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=1
#         )
#         self.subscriber = self.create_subscription(
#             LaserScan,
#             '/autodrive/f1tenth_1/lidar',
#             self.lidar_callback,
#             qos_profile
#         )

#     def lidar_callback(self, msg):
#         # 雷达距离数组
#         ranges = np.array(msg.ranges)

#         # 清洗无效数据（小于最小范围或大于最大范围）
#         valid_ranges = np.where(
#             (ranges > msg.range_min) & (ranges < msg.range_max),
#             ranges,
#             np.inf
#         )

#         # 最近障碍物距离
#         min_distance = np.min(valid_ranges)

#         # 最远距离点对应的角度
#         max_index = np.argmax(valid_ranges)
#         angle = msg.angle_min + max_index * msg.angle_increment

#         # 输出（后期可发布或写入 shared state）
#         # self.get_logger().info(f'Obstacle ahead: {min_distance:.2f} m | Free direction: {angle:.2f} rad')

#         # 可以结构化保存
#         self.perception_output = {
#             'min_obstacle_distance': float(min_distance),
#             'free_direction': float(angle)
#         }

# class CameraPerceptionNode(Node):
#     def __init__(self):
#         super().__init__('camera_perception_node')
#         self.bridge = CvBridge()
#         qos_profile = QoSProfile(
#             reliability=QoSReliabilityPolicy.RELIABLE,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=1
#         )
#         self.subscriber = self.create_subscription(
#             Image,
#             '/autodrive/f1tenth_1/front_camera',
#             self.camera_callback,
#             qos_profile
#         )

#     def camera_callback(self, msg):
#         try:
#             self.get_logger().info("Camera callback triggered.")
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
#             # cv2.namedWindow("Camera View", cv2.WINDOW_NORMAL)
#             print('img:', np.mean(cv_image))  # Print mean pixel value for debugging
#             cv2.imshow("Camera View", cv_image)
#             cv2.waitKey(1)
#             cv2.imwrite('camera_image.jpg', cv_image)  # Save image for debugging

#         except Exception as e:
#             self.get_logger().error(f'Failed to convert image: {e}')
#         return


# def main(args=None):
#     rclpy.init(args=args)

#     lidar_node = LidarPerceptionNode()
#     cam_node = CameraPerceptionNode()

#     executor = MultiThreadedExecutor()
#     executor.add_node(lidar_node)
#     executor.add_node(cam_node)

#     try:
#         executor.spin()

#     finally:
#         lidar_node.destroy_node()
#         cam_node.destroy_node()
#         # cv2.destroyAllWindows()
#         rclpy.shutdown()
#         # cv2.destroyAllWindows()

#     # rclpy.spin(node)
#     # node.destroy_node()
#     # rclpy.shutdown()

if __name__ == '__main__':
    main()
