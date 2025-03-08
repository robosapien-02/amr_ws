source install/setup.bash

ros2 run trajectory_visualization trajectory_publisher_saver

#to send custom odom interface data or you can run turtle3_gazebo to publish odom data.
ros2 topic pub /odom nav_msgs/msg/Odometry '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, child_frame_id: "base_link", pose: {pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}'

ros2 service call /save_trajectory trajectory_visualization/srv/SaveTrajectory "{filename: 'trajectory.csv', duration: 5.0}"

#enter the path of your system i have not maded dynamic at this point so add the path to locate the csv file
ros2 run trajectory_visualization trajectory_reader_publisher --ros-args -p trajectory_file:=/full/path/to/trajectory.csv

#start rviz to visualize the data
rviz2 - set odom and markerarray if not previously done
