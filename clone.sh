# cd src/
# git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
# cd ..
# source /opt/ros/humble/setup.bash
#rosdep init
# rosdep update
# rosdep install --from-paths src --ignore-src -y
# colcon build --symlink-install --packages-select rplidar_ros
# source install/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc
