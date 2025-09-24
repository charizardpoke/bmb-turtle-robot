-- my_cartographer.lua
-- Cartographer ROS 2D (LaserScan) + external wheel odom
-- Frames: map -> odom -> base_footprint
-- LiDAR topic is remapped in your launch to "scan"

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- === Frames ===
  map_frame = "map",
  tracking_frame = "base_footprint",       -- or "base_link" if you prefer
  published_frame = "base_footprint",      -- pose is for the robot base frame
  odom_frame = "odom",

  -- We already have /odom from diff_drive_controller
  provide_odom_frame = false,              -- do NOT create another odom
  publish_frame_projected_to_2d = true,    -- keep base on the ground plane
  publish_to_tf = true,

  -- === Sensor usage ===
  use_odometry = true,     -- consume /odom
  use_nav_sat = false,
  use_landmarks = false,

  -- We're using a single 2D LaserScan
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- === Timing / sampling ===
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 0.05,
  trajectory_publish_period_sec = 0.05,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- Use 2D trajectory builder
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2   -- small SBC-friendly

-- === 2D builder tuning (safe defaults for RPLIDAR-class sensors) ===
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.10
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 4.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Real-time correlative scan matching helps without IMU (CPU cost)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = 0.2

-- Ceres scan matcher weights (reasonable indoor defaults)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0

-- Motion filter: drop nearly-identical scans
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.087  -- ~5 deg

-- Submap / grid options
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"

-- === Pose graph / loop closure ===
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1.0e5

return options
