import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf2_ros
import time


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.get_logger().info('initial_pose_publisher started')
        self.pose_published = False

        # tf buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create client for AMCL lifecycle get_state service
        self.client = self.create_client(GetState, '/amcl/get_state')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /amcl/get_state service...')

        # Timer to check amcl state
        self.timer = self.create_timer(1.0, self.check_amcl_state)

        # Publisher for initial pose
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def check_amcl_state(self):
        if self.pose_published:
            return

        request = GetState.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            state = future.result().current_state
            if state.id == State.PRIMARY_STATE_ACTIVE:
                self.get_logger().info('AMCL is active. Publishing initial pose from TF.')
                self.publish_initial_pose_from_tf()
                self.pose_published = True
                self.destroy_timer(self.timer)
            else:
                self.get_logger().info(f'AMCL not active yet. Current state: {state.label}')
        else:
            self.get_logger().error('Failed to get AMCL state.')

    def publish_initial_pose_from_tf(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'odom',       # from
                'base_link',  # to
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'  # very important: should be 'map'

            # Copy position and orientation from the transform
            msg.pose.pose.position = transform.transform.translation
            msg.pose.pose.orientation = transform.transform.rotation

            # Set covariance (some reasonable initial guess)
            msg.pose.covariance = [
                0.25, 0, 0, 0, 0, 0,
                0, 0.25, 0, 0, 0, 0,
                0, 0, 0.0, 0, 0, 0,
                0, 0, 0, 0.0, 0, 0,
                0, 0, 0, 0, 0.0, 0,
                0, 0, 0, 0, 0, 0.0685389
            ]

            self.pub.publish(msg)
            self.get_logger().info('Initial pose published from odom → base_link TF.')

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not lookup transform from odom to base_link: {str(e)}")



def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
