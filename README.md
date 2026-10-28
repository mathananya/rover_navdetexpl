visual preset accuracy vs high/medium

### WINDOW 0: Do this 
```
mkdir -p ~/maps_$(date +%F)
```

### WINDOW 1: CAMERA

```
ros2 launch realsense2_camera rs_launch.py \
depth_module.depth_profile:=640x480x30 \
rgb_camera.color_profile:=640x480x30 \
align_depth.enable:=true \
pointcloud.enable:=false
```
#### Set pointcloud.enable:=true when creating laserscan


### WINDOW 2: CAMERA TO ROBOT TRANSFORM
#### Height of camera is 46cm rn (change if necessary)
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
approx_sync:=true \
qos:=1 \
queue_size:=30 \
wait_imu_to_init:=false \
use_sim_time:=false \
rtabmap_viz:=true \
rviz:=true \
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
¡New Version! Should ideally now be able to save multiple map checkpoints. Given a super long timeout so hopefully the saver should work now
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
```
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$HOME/my_map.yaml
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p autostart:=true -p node_names:=['map_server']
```

# ERRORS
[rgbd_odometry-1] [INFO] [1792689192.109173262] [rtabmap.rgbd_odometry]: Odom: quality=322, std dev=0.002532m|0.019285rad, update time=0.103492s delay=0.174006s
[rgbd_odometry-1] [INFO] [1792689192.241697466] [rtabmap.rgbd_odometry]: Odom: quality=319, std dev=0.003001m|0.018426rad, update time=0.102463s delay=0.172137s
[rgbd_odometry-1] [INFO] [1792689192.378968826] [rtabmap.rgbd_odometry]: Odom: quality=338, std dev=0.002597m|0.019081rad, update time=0.105298s delay=0.175018s
[rgbd_odometry-1] [INFO] [1792689192.511892623] [rtabmap.rgbd_odometry]: Odom: quality=330, std dev=0.002984m|0.019285rad, update time=0.104095s delay=0.173550s
[rgbd_odometry-1] [INFO] [1792689192.644079442] [rtabmap.rgbd_odometry]: Odom: quality=345, std dev=0.002556m|0.018652rad, update time=0.102035s delay=0.171346s
[rgbd_odometry-1] [WARN] [1792689192.746159532] [rtabmap.rgbd_odometry]: Dropping image/scan data with stamp 1792689192.674307 (delay=0.071839). Something is wrong because the clock difference with the previous topic received (0.011339s) is much lower than the expected one (0.033597s) estimated from the topic stamps (previous stamp=1792689192.640710). If you are processing a large bag with flaky replaying delay, consider setting parameter "always_process_most_recent_frame:=false" to avoid aggressively dropping data.

[rgbd_odometry-1] [ WARN] (2026-10-22 22:42:59.305) OdometryF2M.cpp:642::computeTransform() Registration failed: "Too low inliers after bundle adjustment: 19<20" (guess=xyz=-0.000837,-0.000609,0.000708 rpy=0.000228,0.000218,0.000512)
[rgbd_odometry-1] [ WARN] (2026-10-22 22:42:59.306) OdometryF2M.cpp:325::computeTransform() Failed to find a transformation with the provided guess (xyz=-0.000837,-0.000609,0.000708 rpy=0.000228,0.000218,0.000512), trying again without a guess.
[rgbd_odometry-1] [ WARN] (2026-10-22 22:42:59.461) OdometryF2M.cpp:652::computeTransform() Trial with no guess succeeded!
[rgbd_odometry-1] [INFO] [1792689179.468967182] [rtabmap.rgbd_odometry]: Odom: quality=98, std dev=0.003691m|0.023508rad, update time=0.263589s delay=0.334408s
[rgbd_odometry-1] [ WARN] (2026-10-22 22:42:59.568) OdometryF2M.cpp:642::computeTransform() Registration failed: "Not enough inliers 7/20 (matches=116) between -1 and 2645" (guess=xyz=0.068615,0.005642,-0.014183 rpy=0.000058,-0.118331,-0.003690)
[rgbd_odometry-1] [ WARN] (2026-10-22 22:42:59.568) OdometryF2M.cpp:325::computeTransform() Failed to find a transformation with the provided guess (xyz=0.068615,0.005642,-0.014183 rpy=0.000058,-0.118331,-0.003690), trying again without a guess.
[rgbd_odometry-1] [ WARN] (2026-10-22 22:42:59.726) OdometryF2M.cpp:652::computeTransform() Trial with no guess succeeded!


[rgbd_odometry-1] [WARN] [1792774764.229470316] []: TF_OLD_DATA ignoring data from the past for frame odom at time 317.494000 according to authority Authority undetectable
[rgbd_odometry-1] Possible reasons are listed at http://wiki.ros.org/tf/Errors%20explained
[rtabmap-2] [WARN] [1792774764.256547129] []: TF_OLD_DATA ignoring data from the past for frame odom at time 317.494000 according to authority Authority undetectable






[realsense2_camera_node-1] [WARN] [1792775676.522154727] [camera.camera]: No stream match for pointcloud chosen texture Process - Color

56s, depth=1792775672.226451s). You may want to set approx_sync_max_interval lower than 0.02s to reject spurious bad synchronizations or use approx_sync=false if streams have all the exact same timestamp.
[rgbd_odometry-1] [INFO] [1792775672.459324235] [rtabmap.rgbd_odometry]: Odom: quality=103, std dev=0.002694m|0.019285rad, update time=0.136543s delay=0.266466s


RVIZ running pointcloud errotrs -- launch rtab in localisation mode

rgbd_odometry-1] [INFO] [1792777138.972442626] [rtabmap.rgbd_odometry]: Odom: quality=0, std dev=0.000000m|0.000000rad, update time=0.247717s delay=0.357750s
[rtabmap-2] [ERROR] (2026-10-23 23:08:58.980) Rtabmap.cpp:1416::process() RGB-D SLAM mode is enabled, memory is incremental but no odometry is provided. Image 0 is ignored!



launching params file error
```
LocalPlanner
[controller_server-1] [INFO] [1792777312.369788627] [controller_server]: Setting transform_tolerance to 0.100000
[controller_server-1] [ERROR] [1792777312.417339595] [controller_server]: Couldn't load critics! Caught exception: No critics defined for FollowPath
[controller_server-1] [ERROR] [1792777312.434657210] [controller_server]: Caught exception in callback for transition 10
[controller_server-1] [ERROR] [1792777312.434836294] [controller_server]: Original error: Couldn't load critics! Caught exception: No critics defined for FollowPath
[controller_server-1] [WARN] [1792777312.434906679] [controller_server]: Callback returned ERROR during the transition: configure
[controller_server-1] [FATAL] [1792777312.434937087] [controller_server]: Lifecycle node controller_server does not have error state implemented
[lifecycle_manager-11] [ERROR] [1792777312.440716038] [lifecycle_manager_navigation]: Failed to change state for node: controller_server
[lifecycle_manager-11] [ERROR] [1792777312.440925849] [lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
```
