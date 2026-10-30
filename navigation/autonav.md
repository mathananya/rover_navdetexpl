## How to Run Mapping (rtabmap_tmux.sh)

```
chmod +x ~/rtabmap_nav_tmux.sh
~/rtabmap_nav_tmux.sh
```

### WINDOW 1: CAMERA

```
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640x480x30 \
  rgb_camera.color_profile:=640x480x30 \
  align_depth.enable:=true \
  pointcloud.enable:=true
```

### WINDOW 2: CAMERA TO ROBOT TRANSFORM
Height of camera is 46cm rn (change if necessary)
```
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.46 --frame-id base_link --child-frame-id camera_link
```


### WINDOW 3: RTAB-MAP (VISUAL ODOMETRY AND MAPPING)

```
ros2 launch rtabmap_launch rtabmap.launch.py \
  frame_id:=base_link \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  approx_sync:=true qos:=1 queue_size:=30 \
  wait_imu_to_init:=false use_sim_time:=false \
  rtabmap_viz:=false rviz:=false \
  localization:=true \
  database_path:=$HOME/rtabmap_date.db \
  args:="--Grid/RangeMax 4.0 --Grid/CellSize 0.05 --Grid/MaxObstacleHeight 1.5"
```
Change the database file name, every time you do a mapping. Otherwise, it will overwrite the files.


### WINDOW 4+5: VERIFICATION

```
ros2 topic list | grep odom      # note the odom topic name, likely /rtabmap/odom
```
```
ros2 run tf2_tools view_frames   # confirm map -> odom -> base_link -> camera_link
```

### WINDOW 6: Nav2

```
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=$HOME/nav2_params_v1.yaml use_sim_time:=false
```

### OFF-BOARD: RViz

Visualise on your PC, not on the Jetson!
```
ros2 launch nav2_bringup rviz_launch.py
```
