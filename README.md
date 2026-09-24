
### WINDOW 1: CAMERA

```ros2 launch realsense2_camera rs_launch.py \
depth_module.profile:=640x480x30 \
rgb_camera.profile:=640x480x30 \
align_depth.enable:=true \
pointcloud.enable:=false```


### WINDOW 2: CAMERA TO ROBOT TRANSFORM

```ros2 run tf2_ros static_transform_publisher \
--x 0 --y 0 --z 0.2 --frame-id base_link --child-frame-id camera_link```


### WINDOW 3: RTAB-MAP (VISUAL ODOMETRY AND MAPPING)

```ros2 launch rtabmap_launch rtabmap.launch.py \
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
args:="--delete_db_on_start --Grid/RangeMax 4.0 --Grid/CellSize 0.05 --Grid/MaxObstacleHeight 1.5"```


### VERIFICATION

```ros2 topic hz /rtabmap/odom
ros2 topic info /rtabmap/map --verbose```


### EXPORT MAP

```ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args -r map:=/rtabmap/map```
