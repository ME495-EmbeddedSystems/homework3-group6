# Group 6 Homework 3: Part 2
Authors: 
- Luke Batteas
- Rohan Kota
- Leo Chen
- Zach Alves
- Aditya Naie

## `moveit_api` Package

This package allows you to easily generate and execute motion plans. It also allows you to place boxes in the planning scene.

### Planning a path

You can plan a path by calling the `/plan` service.

The request type is a `moveit_interfaces/srv/Plan`, which consists of the following components:
- __start_pose__: A `geometry_msgs/Pose` defining the pose from which to begin the motion plan. This is only used if `use_start_pose` is set to True.
- __goal_pose__: A `geometry_msgs/Pose` defining the goal pose for the motion plan.
- __plan_mode__: An `int` that specifies the planning mode. There are 3 options:
    - `0`: Plans to the goal_pose's position only
    - `1`: Plans to the goal_pose's orientation only
    - `2`: Plans to the goal_pose's position and orientation
- __use_start_pose__: A `bool` that specifies whether to start at the current configuration or whether to start at the Pose specified by the `start_pose` field
    - `True`: Generates a plan beginning at `start_pose`
    - `False`: Generates a plan beginning at the current robot configuration
- __plan_only__: A `bool` that specifies whether or not to execute the motion plan immediately after it is generated
    - `True`: Only generates the motion plan. The plan does not get executed.
    - `False`: Generates, then executes the motion plan

__Example Usage:__

```
ros2 service call /plan moveit_interfaces/srv/Plan "{start_pose: {position: {x: 0.1, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, goal_pose: {position: {x: 0.2, y: 0.0, z: 0.2}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, plan_mode: 2, use_start_pose: True, plan_only: False}"
```

<br>

### Executing a motion plan

You can execute a previously computed motion plan by calling the `/execute` service.

The request type is `std_srvs/srv/Empty`.

__Example Usage:__

```
ros2 service call /execute std_srvs/srv/Empty "{}"
```

<br>

### Placing a box in the planning scene

You can plan a path by calling the `/place_box` service.

The request type is a `moveit_interfaces/srv/PlaceBox`, which consists of the following components:
- __x__: A `float` defining the x position of the box.
- __y__: A `float` defining the y position of the box.
- __z__: A `float` defining the z position of the box.
- __x_dim__: A `float` defining the x dimension of the box.
- __y_dim__: A `float` defining the y dimension of the box.
- __z_dim__: A `float` defining the z dimension of the box.

__Example Usage:__

```
ros2 service call /place_box moveit_interfaces/srv/PlaceBox "{x: 1.0, y: 1.0, z: 1.0, x_dim: 0.5, y_dim: 0.5, z_dim: 0.5}"
```
