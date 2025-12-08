include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "lidar",
  published_frame = "f1tenth_1",
  odom_frame = "f1tenth_1_odom",
  provide_odom_frame = true,
  use_odometry = false,         
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.3,  
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 0.05,
  trajectory_publish_period_sec = 0.3,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 9.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 6.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.5   -- 单位 m
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(25.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres 扫描匹配权重（加强抑制漂移）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 15
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 30
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 12

-- Motion Filter（避免重复帧进入，提高效率）
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.02
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

-- Voxel / Adaptive Filters（保留足够点）
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.4
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 80
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 9.

-- 约束 / 回环相关参数
POSE_GRAPH.optimize_every_n_nodes = 25
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 8.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(35.)

-- 优化鲁棒性
POSE_GRAPH.optimization_problem.huber_scale = 5.0
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0  -- 当前没用里程计
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0

-- 采样比率全部保留 1
options.rangefinder_sampling_ratio = 1.0
options.odometry_sampling_ratio = 1.0
options.fixed_frame_pose_sampling_ratio = 1.0
options.imu_sampling_ratio = 1.0
options.landmarks_sampling_ratio = 1.0
options.publish_frame_projected_to_2d = false

return options