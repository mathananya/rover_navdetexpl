
### WINDOW 1: CAMERA

```
ros2 launch realsense2_camera rs_launch.py \
depth_module.profile:=640x480x30 \
rgb_camera.profile:=640x480x30 \
align_depth.enable:=true \
pointcloud.enable:=true
```

### WINDOW 2: CAMERA TO ROBOT TRANSFORM
```
ros2 run tf2_ros static_transform_publisher \
--x 0 --y 0 --z 0.46 --frame-id base_link --child-frame-id camera_link
```

### WINDOW 3: LAUNCH RTAB-MAP IN LOCALISATION MODE (AGAINST SAVED DB FROM ABOVE)

```
ros2 launch rtabmap_launch rtabmap.launch.py \
  frame_id:=base_link \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  approx_sync:=true \
  qos:=1 \
  database_path:=$HOME/rtabmap.db \
  rtabmap_viz:=false \
  rviz:=false \
  args:="-localization"
```

### CONVERT TO LASERSCAN

```
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:=/camera/camera/depth/color/points \
  -r scan:=/scan \
  -p target_frame:=base_link \
  -p min_height:=0.05 -p max_height:=0.5 \
  -p range_min:=0.3 -p range_max:=4.0 \
  -p use_inf:=true
```

### TERM 8

```
ros2 launch rover_for_software_unfucked_description sim.launch.py world:=textured slam:=true nav2:=true   rtabmap_extra_args:="--Grid/NormalsSegmentation false --Grid/MaxGroundHeight 0.05 --Grid/NoiseFilteringRadius 0.1 --Grid/NoiseFilteringMinNeighbors 4" odom_mode:=visual
```