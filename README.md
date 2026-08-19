# lidar_scan_publish_subscribe
I have developed a package on ROS2 that starts a lidar on the jetson nano and publishes the data received by the lidar onto a topic named /scan, the topic is then subscribed by a laptop via a ROS_DOMAIN_ID and the data received by the laptop is graphically published onto the RVIZ2 3D visualization simulator.
