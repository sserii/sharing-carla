# carla setup

(terminal 1) $ cd ~/carla && ./CarlaUE4.sh --ros2

*low option: cd ~/carla && ./CarlaUE4.sh -quality-level=Low --ros2
*more low option: cd ~/carla && ./CarlaUE4.sh -quality-level=Low -ResX=640 -ResY=480 --ros2

(terminal 2) $ cd ~/carla/PythonAPI/examples/ros2 && python3 ros2_native.py --file stack.json
(terminal 3) $ rviz2
