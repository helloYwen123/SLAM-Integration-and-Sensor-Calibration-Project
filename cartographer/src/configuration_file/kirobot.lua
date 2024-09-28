-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "robot_imu_link",
  published_frame = "robot_odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_tracked_pose = true,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 2,

  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
 
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2, 
  submap_publish_period_sec = 0.3, 
  pose_publish_period_sec = 5e-3,   
  trajectory_publish_period_sec = 30e-3, 
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.3,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}


TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 15
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 4.5
TRAJECTORY_BUILDER_2D.use_imu_data = true
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1e3 -- default 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 100  -- default 40

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(25.)

-- When performing scan matching with scans found in this window, 
-- a different weight can be chosen for the translational and rotational components.
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.1

-- To avoid inserting too many scans per submaps, 
-- once a motion between two scans is found by the scan matcher, it goes through a motion filter.
--TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.01
--TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

POSE_GRAPH.optimization_problem.huber_scale = 3e2
-------------------------GLOBAL SLAM ----------------------------
POSE_GRAPH.optimize_every_n_nodes = 80 --120
POSE_GRAPH.constraint_builder.sampling_ratio = 0.55   -- default 0.3
POSE_GRAPH.global_sampling_ratio= 0.01 -- default 0.003
POSE_GRAPH.constraint_builder.min_score = 0.75
MAP_BUILDER.num_background_threads = 16 -- up to the maxnum of core
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(25.)
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 5.

-- set these weights depending on how much we trust odometry, 
-- odometry from wheel encoders often has a high uncertainty in rotation. 
-- In this case, the rotation weight can be reduced, even down to zero.
--POSE_GRAPH.optimization_problem.odometry_translation_weight
--POSE_GRAPH.optimization_problem.odometry_rotation_weight
------------------------------------------------------------------


-------------------------LOCAL SLAM ----------------------------
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120 
--------------------------------------------------------------------





return options
