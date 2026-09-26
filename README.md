visual preset accuracy vs high/medium

447274,0.971350,2.851915)
[rgbd_odometry-1] [ WARN] (2026-09-26 16:19:57.673) OdometryF2M.cpp:319::computeTransform() Failed to find a transformation with the provided guess (xyz=1.130603,0.557495,-0.453866 rpy=2.447274,0.971350,2.851915), trying again without a guess.
[rgbd_odometry-1] [ WARN] (2026-09-26 16:19:57.769) OdometryF2M.cpp:616::computeTransform() Trial with no guess still fail.
[rgbd_odometry-1] [ WARN] (2026-09-26 16:19:57.769) OdometryF2M.cpp:626::computeTransform() Registration failed: "Not enough inliers 0/20 (matches=44) between -1 and 76" (guess=xyz=1.130603,0.557495,-0.453866 rpy=2.447274,0.971350,2.851915)
[rgbd_odometry-1] [INFO] [1790419797.770874157] [rtabmap.rgbd_odometry]: Odom: quality=0, std dev=0.000000m|0.000000rad, update time=0.149885s delay=0.216484s
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[rtabmap-2] [INFO] [1790419797.788268019] [rclcpp]: signal_handler(SIGINT/SIGTERM)
[rgbd_odometry-1] [INFO] [1790419797.790255323] [rclcpp]: signal_handler(SIGINT/SIGTERM)




### WINDOW 1: CAMERA

```
ros2 launch realsense2_camera rs_launch.py \
depth_module.profile:=640x480x30 \
rgb_camera.profile:=640x480x30 \
align_depth.enable:=true \
pointcloud.enable:=false
```


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
