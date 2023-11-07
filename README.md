# ME 495 Homework 3 Part II, Group 6
Authors: 
- Luke Batteas
- Rohan Kota
- Leo Chen
- Zach Alves
- Aditya Nair

## Setup

1. Ensure that the move_group_interface package is built and sourced. Code examples can be see in the `move_group_test` node.
2. Import the `RobotModel` and`MoveGroupInterface`. 
 - The `RobotModel` contains details about which joints should be moved.
    - The group name and associated joints must be specified.
    - Optionally, a default end effector and base link can be specificed
    - Optionally, a namespace for the joint_states can be specified
- The `MoveGroupInterface` is the API and contains all associated functions
    - Upon creation, a RobotModel and a Node must be passed. The node is used to create clients and subscriptions.
    - Optionally, a namespace for move_group can be specified
    - Optionally, a time to wait for service can be specified. The default is 3 seconds.
    An 

3. There are two required parameters to set before motion planning. The `Workspace` and the `planning_pipeline_id`. An example of code is shown below.
    ```
    self.API.setWorkspaceParamaters(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0)
    self.API.setPlanningPipelineID("move_group")
    ```
    - The Workspace determines in what region the robot can plan in. 
        - The region is cube specified by the diagonal corners.
        - If no frame_id is specified in the call and a default base link was specified, it will assume the base link is the proper frame.
4. The API is now set up and can be used to make motion plan requests. There are a variety of other parameters that can be set. 
    These parameters are shown below, along with their default values.
    ```
    - planning_pipeline_id_ = "" 
    - planner_id_ = ""
    - workspace_parameters_ = None
    - can_look_ = False
    - look_around_attempts_ = 0
    - can_replan_ = False
    - replan_delay_ = 2.0
    - replan_attempts_ = 1
    - goal_joint_tolerance_ = 1e-4
    - goal_position_tolerance_ = 1e-4
    - goal_orientation_tolerance_ = 1e-3
    - allowed_planning_time_ = 5.0
    - num_planning_attemps_ = 1
    - max_velocity_scaling_factor_ = 0.1
    - max_acceleration_scaling_factor_ = 0.1 
    ```

## Constraints

1. Prior to making a motion plan request, goal constraints must be set. There are four types of goal constraints.
 - Position Constrains : A goal on the postion of a link
 - Orientation Constraint : A goal on the orientation of a link
 - Joint Constraint : A goal on the joint position
 - Visibility Constraint : A goal on a sensor's field of view. This type of constraint is not currently supported.
 Additionally, a Pose Constraint is specified in code as both a position and an orientation constraint.

2. Add constraints by making the assocaited API call. 
   Additionally, a joint constraint can be added from a link pose and a pose constraint can be added from a joint state.
   For position, orientation, and pose constraints, a link must be specified. If not passed and a default end effector link is present, it will assume the end effector link.
   Tolerances and weights may be additionally specified with the call. Otherwise it will take on their default value as shown in the parameters given above (with the weight being 1.0).

3. Once at least goal constraint is specified, a motion plan request can be called. 



## Motion

1. Make a motion plan request by calling `makeMotionPlanRequest`. This function has several optional parameters.
- Optionally, a bool `plan_only` can be specified true or false. It defaults to true. If set to false, the robot will execute the trajectory after the planning. If this happens, the start state is set to the current state of the robot. 
- Optionally, pass a separate start state. This will override existing specified start states. This must be passed as a RobotState message. If no start state is specified either before or in this call, the start state is assumed to be the current state, which is collected from the joint states being published. 
- Optionally, an end effector pose can be specified when calling this function. This places a pose constraint as a goal constraint for the call. 
- Optionally, a bool `clean` will erase the start state and goal constraints once the action is called if set to true. It defaults to true.

2. If the `makeMotionPlanRequest` returned trajectory is saved, it can later be executed with `executeTrajectory`. The trajectory is the only argument passed.

## Collision Objects
1. Collision objects can be added to the scene with `addCollisionObject`. The two arguments required are a SolidPrimitive shape and a stamped pose of the position where the objecet is located. An example of code is shown below.
```
self.box = SolidPrimitive()
self.box.type = 1
self.box.dimensions = [0.5,0.5,0.5]

self.box_pose = PoseStamped()
self.box_pose.header.frame_id = "panda_link0"
self.box_pose.pose.position.x = 1.0
self.box_pose.pose.position.y = 1.0
self.box_pose.pose.position.z = 0.0
 await self.API.addCollisionObject(self.box, self.box_pose)await self.API.addCollisionObject(self.box, self.box_pose)
```
