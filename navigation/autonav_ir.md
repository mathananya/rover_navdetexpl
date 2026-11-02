```
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640x480x30 \
  enable_color:=false \
  enable_infra1:=false \
  enable_infra2:=false \
  align_depth.enable:=false \
  pointcloud.enable:=false \
  initial_reset:=true
```

```
ros2 run tf2_ros static_transform_publisher \
  --x 0.28 --y -0.16 --z 0.42 --roll 0 --pitch 0.0698 --yaw 0 --frame-id base_link --child-frame-id camera_link
```

```
ros2 run rtabmap_util point_cloud_xyz --ros-args \
  -r depth/image:=/camera/camera/depth/image_rect_raw \
  -r depth/camera_info:=/camera/camera/depth/camera_info \
  -r cloud:=/camera/depth/points \
  -p decimation:=2 -p max_depth:=4.0 -p voxel_size:=0.05 -p qos:=1
```

```
ros2 launch rtabmap_launch rtabmap.launch.py \
  frame_id:=base_link \
  icp_odometry:=true visual_odometry:=false \
  depth:=false subscribe_rgb:=false \
  subscribe_scan_cloud:=true scan_cloud_topic:=/camera/depth/points \
  qos:=1 use_sim_time:=false \
  rtabmap_viz:=false rviz:=false \
  localization:=true \
  database_path:=$HOME/rtabmap_0211_ir_v1.db \
  odom_args:="--Reg/Strategy 1 --Reg/Force3DoF true --Icp/PointToPlane true --Icp/VoxelSize 0.05 --Icp/MaxCorrespondenceDistance 0.2" \
  args:="--Reg/Strategy 1 --Reg/Force3DoF true --Icp/PointToPlane true --Icp/VoxelSize 0.05 --Grid/Sensor 0 --Grid/RangeMax 4.0 --Grid/CellSize 0.05 --Grid/MaxObstacleHeight 1.5"
```

```
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=$HOME/nav2_params_smac_2d.yaml use_sim_time:=false
```
