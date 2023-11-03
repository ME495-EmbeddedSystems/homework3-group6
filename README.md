# `moveit_api` Package

Authors: 
- Luke Batteas
- Rohan Kota
- Leo Chen
- Zach Alves
- Aditya Naie

<br>

This package allows you to easily generate and execute motion plans. It also allows you to place boxes in the planning scene.

## Launching the node

You can run the moveit api using the following command:

```
ros2 launch moveit_api moveit_api.launch.xml "robot_param_file:=panda_params.yaml"
```

<br>

### robot_param_file argument:

You can specify the parameter file for the robot through the `robot_param_file` argument. The `moveit_api.launch.xml` launch file will launch an api for the Panda Arm by default if this argument is not specified.
- __Panda Arm:__ `ros2 launch moveit_api moveit_api.launch.xml "robot_param_file:=panda_params.yaml"`
- __Interbotix PX100 Arm:__ `ros2 launch moveit_api moveit_api.launch.xml "robot_param_file:=interbotix_params.yaml"`

<br>

__Required Parameters__:
- `namespace`: The namespace used by the robot (a blank string indicates no namespace).
- `base_frame_id`: The name of the robot's base frame that all the specified poses are relative to.
- `end_effector_link`: The end effector link that we are planning for.
- `group_name`: The name of the planning group.
- `use_joint_constraints`: Set to `True` if you want to use joint constraints to construct the motion plan. Set to `False` to use position and orientation constraints instead.

<br>

Here is an example of the param file for the Panda Arm:

```yaml
/moveit_api:
  ros__parameters:
    namespace: ''
    base_frame_id: 'panda_link0'
    end_effector_link: 'panda_link8'
    group_name: 'panda_manipulator'
    use_joint_constraints: False
```

<br>

## Planning a path

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

## Executing a motion plan

You can execute a previously computed motion plan by calling the `/execute` service.

The request type is `std_srvs/srv/Empty`.

__Example Usage:__

```
ros2 service call /execute std_srvs/srv/Empty "{}"
```

<br>

## Placing a box in the planning scene

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
