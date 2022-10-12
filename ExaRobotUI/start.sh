killall -9 python3
killall -9 _ros2_daemon
killall -9 realsense2_camera_node

../script/kill_nav.sh
python3 main.py
