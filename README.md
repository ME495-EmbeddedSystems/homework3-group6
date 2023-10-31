# Group 6 Homework 3: Part 2
Authors: 
- Luke Batteas
- Rohan Kota
- Leo Chen
- Zach Alves
- Aditya Naie

ros2 service call /plan moveit_interfaces/srv/Plan "{start_pose: {position: {x: 0.2, y: 0.0, z: 0.2}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, goal_pose: {position: {x: 0.2, y: 0.0, z: 0.3}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, use_start_pose: True}"