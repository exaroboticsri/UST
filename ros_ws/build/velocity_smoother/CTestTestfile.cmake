# CMake generated Testfile for 
# Source directory: /home/jw/robot/ros_ws/src/ExaRobot_ROS2/velocity_smoother
# Build directory: /home/jw/robot/ros_ws/build/velocity_smoother
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_test_translational_smoothing.py "/usr/bin/python3" "-u" "/opt/ros/galactic/share/ament_cmake_test/cmake/run_test.py" "/home/jw/robot/ros_ws/build/velocity_smoother/test_results/velocity_smoother/test_test_translational_smoothing.py.xunit.xml" "--package-name" "velocity_smoother" "--output-file" "/home/jw/robot/ros_ws/build/velocity_smoother/launch_test/test_test_translational_smoothing.py.txt" "--command" "/usr/bin/python3" "-m" "launch_testing.launch_test" "/home/jw/robot/ros_ws/src/ExaRobot_ROS2/velocity_smoother/test/test_translational_smoothing.py" "--junit-xml=/home/jw/robot/ros_ws/build/velocity_smoother/test_results/velocity_smoother/test_test_translational_smoothing.py.xunit.xml" "--package-name=velocity_smoother")
set_tests_properties(test_test_translational_smoothing.py PROPERTIES  TIMEOUT "30" WORKING_DIRECTORY "/home/jw/robot/ros_ws/build/velocity_smoother" _BACKTRACE_TRIPLES "/opt/ros/galactic/share/ament_cmake_test/cmake/ament_add_test.cmake;124;add_test;/opt/ros/galactic/share/launch_testing_ament_cmake/cmake/add_launch_test.cmake;125;ament_add_test;/home/jw/robot/ros_ws/src/ExaRobot_ROS2/velocity_smoother/CMakeLists.txt;65;add_launch_test;/home/jw/robot/ros_ws/src/ExaRobot_ROS2/velocity_smoother/CMakeLists.txt;0;")
