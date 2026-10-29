#!/usr/bin/env bash
# Creates a tmux session with 6 panes (3 rows x 2 columns) and TYPES the
# commands into each pane WITHOUT pressing Enter. Go to each pane and hit
# Enter when you're ready to run it.
#
# Layout:
#   +-----------------+-----------------+
#   | 1 realsense     | 2 static tf     |
#   +-----------------+-----------------+
#   | 3 rtabmap       | 4 odom hz       |
#   +-----------------+-----------------+
#   | 5 map info      | 6 map saver     |
#   +-----------------+-----------------+

SESSION="rtabmap"

# If the session already exists, just attach to it
if tmux has-session -t "$SESSION" 2>/dev/null; then
    exec tmux attach -t "$SESSION"
fi

# Create the session and capture pane IDs (stable, unlike pane indexes)
P1=$(tmux new-session -d -s "$SESSION" -x 220 -y 60 -P -F '#{pane_id}')

# 3 rows: split the left column vertically twice, then even them out
P3=$(tmux split-window -v -t "$P1" -P -F '#{pane_id}')
P5=$(tmux split-window -v -t "$P3" -P -F '#{pane_id}')
tmux select-layout -t "$SESSION" even-vertical

# 2 columns: split each row horizontally
P2=$(tmux split-window -h -t "$P1" -P -F '#{pane_id}')
P4=$(tmux split-window -h -t "$P3" -P -F '#{pane_id}')
P6=$(tmux split-window -h -t "$P5" -P -F '#{pane_id}')

# Type commands (-l = literal). No "Enter" is sent, so nothing runs.
# Single quotes keep $HOME unexpanded here; it expands when you run it in the pane.

tmux send-keys -t "$P1" -l 'ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=640x480x30 rgb_camera.color_profile:=640x480x30 align_depth.enable:=true pointcloud.enable:=false'

tmux send-keys -t "$P2" -l 'ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.46 --frame-id base_link --child-frame-id camera_link'

tmux send-keys -t "$P3" -l 'ros2 launch rtabmap_launch rtabmap.launch.py frame_id:=base_link rgb_topic:=/camera/camera/color/image_raw depth_topic:=/camera/camera/aligned_depth_to_color/image_raw camera_info_topic:=/camera/camera/color/camera_info approx_sync:=true qos:=1 queue_size:=30 wait_imu_to_init:=false use_sim_time:=false rtabmap_viz:=false rviz:=false database_path:=$HOME/rtabmap_date.db args:="--delete_db_on_start --Grid/RangeMax 4.0 --Grid/CellSize 0.05 --Grid/MaxObstacleHeight 1.5"'

tmux send-keys -t "$P4" -l 'ros2 topic hz /rtabmap/odom'

tmux send-keys -t "$P5" -l 'ros2 topic info /rtabmap/map --verbose'

tmux send-keys -t "$P6" -l 'ros2 run nav2_map_server map_saver_cli -f ~/my_map --ros-args -r map:=/rtabmap/map'

# Start with the first pane focused, then attach
tmux select-pane -t "$P1"
tmux attach -t "$SESSION"
