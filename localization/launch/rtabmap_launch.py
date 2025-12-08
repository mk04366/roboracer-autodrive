from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_viz = LaunchConfiguration('use_viz')
    use_viz_arg = DeclareLaunchArgument(
        'use_viz', default_value='true',
        description='Start rtabmap_viz GUI'
    )

    pkg_share = get_package_share_directory('localization')
    cfg = os.path.join(pkg_share, 'config', 'rtabmap_config.yaml')

    # 你的激光话题
    remap_scan = [('scan', '/autodrive/f1tenth_1/lidar')]

    # ① ICP 里程计：根据激光估计 odom→base（发布 /odom + TF）
    icp_odom = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[cfg],
        remappings=remap_scan
    )

    # ② RTAB-Map 主节点（用上面的里程计 + 同一份激光）
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[cfg],
        remappings=remap_scan
    )

    # ③ 可选可视化
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        condition=None  # 用下面 if 控制
    )

    nodes = [use_viz_arg, icp_odom, rtabmap]
    # 简单处理：如果 use_viz!=false 就加 viz
    #（LaunchConfiguration 的真值判断在纯 Python 不方便，这里用环境约定：你想关掉就启动时传 use_viz:=false）
    if os.environ.get('RTABMAP_USE_VIZ', '1') != '0':
        nodes.append(rtabmap_viz)

    # --- 可选：如果系统里没有广播 base->lidar 的静态TF，请取消注释以下两段 ---
    # nodes += [
    #     Node(
    #         package='tf2_ros', executable='static_transform_publisher',
    #         name='base_to_lidar_static',
    #         arguments=['0.2','0','0.1','0','0','0','f1tenth_1','lidar']  # 改成你的真实外参
    #     ),
    # ]
    # 千万不要发布 odom->base 的静态 TF，因为 icp_odometry 会发布动态的！

    return LaunchDescription(nodes)