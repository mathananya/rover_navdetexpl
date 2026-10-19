visual preset accuracy vs high/medium

### WINDOW 1: CAMERA

```
ros2 launch realsense2_camera rs_launch.py \
depth_module.profile:=640x480x30 \
rgb_camera.profile:=640x480x30 \
align_depth.enable:=true \
pointcloud.enable:=false
```
#### Set pointcloud.enable:=true when creating laserscan


### WINDOW 2: CAMERA TO ROBOT TRANSFORM

```
ros2 run tf2_ros static_transform_publisher \
--x 0 --y 0 --z 0.2 --frame-id base_link --child-frame-id camera_link
```


### WINDOW 3: RTAB-MAP (VISUAL ODOMETRY AND MAPPING)

```
ros2 launch rtabmap_launch rtabmap.launch.py \
frame_id:=base_link \
rgb_topic:=/camera/camera/color/image_raw \
depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
camera_info_topic:=/camera/camera/color/camera_info \
approx_sync:=true \
qos:=1 \
queue_size:=30 \
wait_imu_to_init:=false \
use_sim_time:=false \
rtabmap_viz:=false \
rviz:=false \
database_path:=$HOME/rtabmap.db \
args:="--delete_db_on_start --Grid/RangeMax 4.0 --Grid/CellSize 0.05 --Grid/MaxObstacleHeight 1.5"
```


### VERIFICATION

```
ros2 topic hz /rtabmap/odom
ros2 topic info /rtabmap/map --verbose
```


### EXPORT MAP

```
ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args -r map:=/rtabmap/map
```
¡New Version! Should ideally now be able to save multiple map checkpoints.
```
ros2 service call /rtabmap/publish_map rtabmap_msgs/srv/PublishMap "{graph_only: false, optimized: true, global_map: true}"
sleep 1.5
ros2 run nav2_map_server map_saver_cli -f ~/maps/map_$(date +%Y%m%d_%H%M%S) --ros-args -r map:=/rtabmap/map -p save_map_timeout:=10.0
```

### LAUNCH RTAB-MAP IN LOCALISATION MODE (AGAINST SAVED DB FROM ABOVE)

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


#### nav2_params.yaml (no amcl params)
```
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_link
      plugins: ["static_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.4
```


#### using navigation_launch.py instead of bringup_launch.py because amcl not needed
```
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=$HOME/nav2_params.yaml \
  use_sim_time:=false \
  map_subscribe_transient_local:=true
```

