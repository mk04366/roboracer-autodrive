import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path # 用于订阅位姿和路径
from geometry_msgs.msg import Twist # 用于发布控制指令
# import numpy as np # 导入 numpy 用于矩阵计算

class MPCCController(Node):
    def __init__(self):
        super().__init__('mpcc_controller')
        
        # 1. 声明和获取参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pose_topic', '/odom'),
                ('path_topic', '/global_path'),
                ('control_topic', '/cmd_vel'),
                ('horizon_length', 10),
                ('time_step', 0.1),
                ('wheelbase', 0.330),
                # ... 获取其他权重参数
            ]
        )
        # 示例：获取参数值
        self._horizon_length = self.get_parameter('horizon_length').value
        self._pose_topic = self.get_parameter('pose_topic').value
        
        self.get_logger().info(f'MPCC Node started. Horizon: {self._horizon_length}, Pose Topic: {self._pose_topic}')

        # 2. 状态存储
        self.current_pose = None
        self.reference_path = None
        
        # 3. 订阅 (Subscribe) - 获取当前位姿 (Odometry)
        self.pose_sub = self.create_subscription(
            Odometry, 
            self._pose_topic, 
            self.odom_callback, 
            10
        )
        
        # 4. 订阅 (Subscribe) - 获取参考路径 (Path)
        self.path_sub = self.create_subscription(
            Path, 
            self.get_parameter('path_topic').value, 
            self.path_callback, 
            1
        )
        
        # 5. 发布 (Publish) - 发布控制指令 (Twist)
        self.control_pub = self.create_publisher(Twist, self.get_parameter('control_topic').value, 10)
        
        # 6. 设置定时器，定期运行 MPCC 求解 (例如 10 Hz)
        self.timer = self.create_timer(self.get_parameter('time_step').value, self.timer_callback)

    def odom_callback(self, msg: Odometry):
        """里程计/位姿回调函数"""
        self.current_pose = msg.pose.pose
        
    def path_callback(self, msg: Path):
        """参考路径回调函数"""
        # 路径通常只需要接收一次或在更新时接收
        self.reference_path = msg.poses
        self.get_logger().info(f"Received path with {len(self.reference_path)} points.")

    def timer_callback(self):
        """定时运行，执行 MPCC 求解和控制发布"""
        if self.current_pose is None:
            self.get_logger().warn("Waiting for current pose (Odometry)...")
            return
            
        if self.reference_path is None or len(self.reference_path) == 0:
            self.get_logger().warn("Waiting for reference path...")
            return

        self.compute_and_publish_control() 

    def compute_and_publish_control(self):
        """
        MPCC 核心逻辑:
        1. 状态提取: 从 self.current_pose 提取 x, y, yaw, 速度。
        2. 路径处理: 查找 self.reference_path 中离当前状态最近的点。
        3. 优化求解: 建立并求解 MPCC 优化问题 (使用 CasADi/IPOPT 等)。
        4. 发布控制: 发布第一个最优控制量。
        """
        
        # --- [ 您的 MPCC 求解器代码将在这里 ] ---
        
        # 示例：发布一个零速度
        cmd = Twist()
        cmd.linear.x = 0.0  # 计算得到的最优线速度
        cmd.angular.z = 0.0 # 计算得到的最优角速度（对应转向角）
        
        self.control_pub.publish(cmd)
        self.get_logger().debug(f"Published control: V={cmd.linear.x:.2f}, W={cmd.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    mpcc_controller = MPCCController()
    rclpy.spin(mpcc_controller)
    mpcc_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()