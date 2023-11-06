import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_interfaces.srv import PlaceBox, Plan
from std_srvs.srv import Empty, SetBool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.msg import PositionIKRequest, CollisionObject, PlanningScene, \
    PlanningSceneComponents, MotionPlanRequest, Constraints, PositionConstraint, \
    OrientationConstraint, BoundingVolume, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetPlanningScene
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from control_msgs.action import GripperCommand
from shape_msgs.msg import SolidPrimitive
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer


class MoveitAPI(Node):

    def __init__(self):
        super().__init__('moveit_api')

        # Declare and get Parameters
        self.declare_parameter('base_frame_id', 'panda_link0')
        self.declare_parameter('end_effector_link', 'panda_link8')
        self.declare_parameter('group_name', 'panda_manipulator')
        self.base_frame_id = self.get_parameter(
            'base_frame_id').get_parameter_value().string_value
        self.end_effector_link = self.get_parameter(
            'end_effector_link').get_parameter_value().string_value
        self.group_name = self.get_parameter(
            'group_name').get_parameter_value().string_value

        # Create Reentrant Callback Group
        self.cbgroup = ReentrantCallbackGroup()

        # Subscriber for joint state messages
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.joint_state_msg = None

        # Create services to generate motion plan, execute motion plan,
        # place box, and open/close grippers
        self.plan = self.create_service(
            Plan, "plan", self.generate_motion_plan, callback_group=self.cbgroup)
        self.execute = self.create_service(
            Empty, "execute", self.execute_trajectory, callback_group=self.cbgroup)
        self.place_box = self.create_service(
            PlaceBox, "place_box", self.place_box_callback, callback_group=self.cbgroup)
        self.move_gripper = self.create_service(
            SetBool, "gripper", self.gripper_callback, callback_group=self.cbgroup)

        # Create service client to comput IK
        self.ik_client = self.create_client(
            GetPositionIK, "compute_ik", callback_group=self.cbgroup)

        # Create action client to generate motion plans
        self.plan_client = ActionClient(self, MoveGroup, "move_action")
        self.motion_plan = None

        # Create action client to execute motion plans
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, "execute_trajectory")

        # Create action client to open/close the Panda's grippers
        self.gripper_client = ActionClient(
            self, GripperCommand, "panda_gripper/gripper_action")

        # Planning scene service client and publisher
        self.scene_client = self.create_client(
            GetPlanningScene, "get_planning_scene")
        self.box_publisher = self.create_publisher(
            PlanningScene, "planning_scene", 10)

        # Set up TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    async def place_box_callback(self, request, response):
        """
        Places a box at any x,y,z location.

        Args
        ----
            request (PlaceBoxRequest): A PlaceBox request object

            response (EmptyResponse): The response object

        Returns
        -------
           An EmptyResponse

        """
        component = PlanningSceneComponents()
        planning_scene = await self.scene_client.call_async(
            GetPlanningScene.Request(components=component))
        scene = planning_scene.scene

        solid_primitive = SolidPrimitive()
        solid_primitive.type = 1
        solid_primitive.dimensions = [
            request.x_dim, request.y_dim, request.z_dim]

        pose = Pose()

        box = CollisionObject()
        box.header.stamp = self.get_clock().now().to_msg()
        box.header.frame_id = self.base_frame_id
        box.id = 'my_box'
        box.pose.position.x = request.x
        box.pose.position.y = request.y
        box.pose.position.z = request.z
        box.pose.orientation.x = 0.0
        box.pose.orientation.y = 0.0
        box.pose.orientation.z = 0.0
        box.pose.orientation.w = 1.0
        box.primitives = [solid_primitive]
        box.primitive_poses = [pose]

        scene.world.collision_objects.append(box)

        self.box_publisher.publish(scene)

        return response

    def build_ik_request(self, pose):
        """
        Build a position IK request.

        Args
        ----
            pose (geometry_msgs/Pose): The desired end effector pose

        Returns
        -------
           A GetPositionIK Request object

        """
        pos_ik_req = PositionIKRequest()
        pos_ik_req.group_name = self.group_name
        pos_ik_req.robot_state.joint_state = self.joint_state_msg
        pos_ik_req.pose_stamped.header.frame_id = self.base_frame_id
        pos_ik_req.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pos_ik_req.pose_stamped.pose.position.x = pose.position.x
        pos_ik_req.pose_stamped.pose.position.y = pose.position.y
        pos_ik_req.pose_stamped.pose.position.z = pose.position.z
        pos_ik_req.pose_stamped.pose.orientation.x = pose.orientation.x
        pos_ik_req.pose_stamped.pose.orientation.y = pose.orientation.y
        pos_ik_req.pose_stamped.pose.orientation.z = pose.orientation.z
        pos_ik_req.pose_stamped.pose.orientation.w = pose.orientation.w
        pos_ik_req.timeout.sec = 5

        get_pos_ik_req = GetPositionIK.Request()
        get_pos_ik_req.ik_request = pos_ik_req

        self.get_logger().info(str(pos_ik_req))

        return get_pos_ik_req

    async def build_plan_request(self, start_state, goal_pose, plan_mode, plan_only=True):
        """
        Build a path planning request.

        Args
        ----
            start_state (sensor_msgs/JointState): The starting joint configuration

            goal_pose (geometry_msgs/Pose): The goal pose for the robot

            plan_mode (int): 0 - position only, 1 - orientation_only, 2 - position and orientation

            plan_only (bool): Determines whether or not the plan gets executed once complete

        Returns
        -------
           A MoveGroup Goal object

        """
        # Create Constraints
        goal_constraints = Constraints()

        if plan_mode == 0 or plan_mode == 2:

            solid_primitive = SolidPrimitive()
            solid_primitive.type = 1
            solid_primitive.dimensions = [2e-3, 2e-3, 2e-3]

            pose = Pose()
            pose.position.x = goal_pose.position.x
            pose.position.y = goal_pose.position.y
            pose.position.z = goal_pose.position.z

            BV = BoundingVolume()
            BV.primitives = [solid_primitive]
            BV.primitive_poses = [pose]

            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = self.base_frame_id
            position_constraint.link_name = self.end_effector_link
            position_constraint.constraint_region = BV
            position_constraint.weight = 1.0

            goal_constraints.position_constraints.append(position_constraint)

        if plan_mode == 1 or plan_mode == 2:
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = self.base_frame_id
            orientation_constraint.link_name = self.end_effector_link
            orientation_constraint.orientation = goal_pose.orientation
            orientation_constraint.absolute_x_axis_tolerance = 1e-3
            orientation_constraint.absolute_y_axis_tolerance = 1e-3
            orientation_constraint.absolute_z_axis_tolerance = 1e-3
            orientation_constraint.weight = 1.0

            goal_constraints.orientation_constraints.append(
                orientation_constraint)

        # Create Motion Plan Request
        req = MotionPlanRequest()
        req.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0
        req.workspace_parameters.header.frame_id = self.base_frame_id
        req.start_state.joint_state = start_state
        req.goal_constraints = [goal_constraints]
        req.pipeline_id = 'move_group'
        req.group_name = self.group_name
        req.num_planning_attempts = 1
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1

        # Create MoveGroup Goal
        plan_request = MoveGroup.Goal()
        plan_request.request = req
        plan_request.planning_options.plan_only = plan_only

        return plan_request

    async def compute_ik(self, pose):
        """
        Compute the inverse kinematic for a given end effector pose.

        Args
        ----
            pose (geometry_msgs/Pose): The goal pose for the end effector

        Returns
        -------
           A GetPositionIK Response object

        """
        # Build the IK Request
        get_pos_ik_req = self.build_ik_request(pose)

        # Call the IK service
        ik_response = await self.ik_client.call_async(get_pos_ik_req)

        return ik_response

    async def generate_motion_plan(self, request, response):
        """
        Generate a motion plan for the robot.

        Args
        ----
            request (PlanRequest): A plan request

        Returns
        -------
           An EmptyResponse object

        """
        # If start_pose is provided, compute the JointState for that pose
        if request.use_start_pose is True:

            ik_response = await self.compute_ik(request.start_pose)

            # If no IK solution is found
            if ik_response.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
                self.get_logger().warn(
                    "No IK solution found for start pose. Starting at current \
                        robot configuration instead.")
                start_joint_state = self.joint_state_msg

            # If IK solution is found, use that JointState as starting JointState
            else:
                start_joint_state = ik_response.solution.joint_state

        # Otherwise use the current JointState
        else:
            start_joint_state = self.joint_state_msg

        # Create the MoveGroup Goal
        plan_req = await self.build_plan_request(
            start_joint_state, request.goal_pose,
            request.plan_mode, request.plan_only)

        # Call the move action client
        future_response = await self.plan_client.send_goal_async(plan_req)
        plan_response = await future_response.get_result_async()
        self.motion_plan = plan_response

        return response

    async def execute_trajectory(self, request, response):
        """
        Execute a previously computed motion plan.

        Args
        ----
            request (Empty): An Empty request

        Returns
        -------
           An EmptyResponse object

        """
        # If a motion plan has been computed
        if self.motion_plan is not None:
            execute_req = ExecuteTrajectory.Goal()
            execute_req.trajectory = self.motion_plan.result.planned_trajectory

            future_response = await self.execute_client.send_goal_async(execute_req)
            await future_response.get_result_async()

        # Otherwise, display an error to the user
        else:
            self.get_logger().error("No motion plan computed.")

        return response

    async def gripper_callback(self, request, response):
        """
        Open or close the gripper of the Interbotix arm.

        Args
        ----
            request (SetBoolRequest): A SetBool Request

        Returns
        -------
        A SetBool Response object

        """
        open = request.data

        # Create the GripperCommand Goal
        gripper_request = GripperCommand.Goal()

        if open is False:
            self.get_logger().info("Closing Gripper")
            gripper_request.command.position = 0.025
            gripper_request.command.max_effort = 10.0
        else:
            self.get_logger().info("Opening Gripper")
            gripper_request.command.position = 0.035

        # Call the move action client
        future_response = await self.gripper_client.send_goal_async(gripper_request)
        await future_response.get_result_async()

        response.success = True

        return response

    def joint_state_callback(self, msg):
        """
        Update the current JointState of the robot.

        Args:
        ----
            msg (JointState): The current JointState of the robot

        """
        self.joint_state_msg = msg


def main(args=None):
    rclpy.init(args=args)

    moveit_api = MoveitAPI()

    rclpy.spin(moveit_api)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    moveit_api.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
