import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_interfaces.srv import PlaceBox, Plan
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.msg import PositionIKRequest, CollisionObject, PlanningScene, PlanningSceneComponents, MotionPlanRequest, Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetPlanningScene
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer


class MoveitAPI(Node):

    def __init__(self):
        super().__init__('moveit_api')

        self.cbgroup = ReentrantCallbackGroup()

        # Subscriber for joint state messages
        self.joint_state_sub = self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.joint_state_msg = None

        # Create services to generate motion plan and place box
        self.plan = self.create_service(Plan, "plan", self.generate_motion_plan, callback_group=self.cbgroup)
        self.place_box = self.create_service(PlaceBox, "place_box", self.place_box_callback, callback_group=self.cbgroup)

        # Create service client to comput IK
        self.ik_client = self.create_client(GetPositionIK, "compute_ik", callback_group=self.cbgroup)

        # Create action client to generate motion plans
        self.plan_client = ActionClient(self, MoveGroup, "move_action")

        # Planning scene service client and publisher
        self.scene_client = self.create_client(GetPlanningScene, "get_planning_scene")
        self.box_publisher = self.create_publisher(PlanningScene, "planning_scene", 10)

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
        planning_scene = await self.scene_client.call_async(GetPlanningScene.Request(components=component))
        scene = planning_scene.scene

        solid_primitive = SolidPrimitive()
        solid_primitive.type = 1
        solid_primitive.dimensions = [request.x_dim, request.y_dim, request.z_dim]

        pose = Pose()

        box = CollisionObject()
        box.header.stamp = self.get_clock().now().to_msg()
        box.header.frame_id = "panda_link0"
        box.id = "my_box"
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
        Builds a position IK request.

        Args
        ----
            pose (geometry_msgs/Pose): The desired end effector pose

        Returns
        -------
           A GetPositionIK Request object

        """
        pos_ik_req = PositionIKRequest()
        pos_ik_req.group_name = 'panda_manipulator'
        pos_ik_req.robot_state.joint_state = self.joint_state_msg
        pos_ik_req.pose_stamped.header.frame_id = 'panda_link0'
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

        return get_pos_ik_req
    
    def build_plan_request(self, start_state, goal_pose, plan_mode, plan_only = True):
        """
        Builds a path planning request.

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
            BV.primitives =  [solid_primitive]
            BV.primitive_poses = [pose]

            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = 'panda_link0'
            position_constraint.link_name = 'panda_link8'
            position_constraint.constraint_region = BV
            position_constraint.weight = 1.0

            goal_constraints.position_constraints.append(position_constraint)

        if plan_mode == 1 or plan_mode == 2:
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = 'panda_link0'
            orientation_constraint.link_name = 'panda_link8'
            orientation_constraint.orientation = goal_pose.orientation
            orientation_constraint.absolute_x_axis_tolerance = 1e-3
            orientation_constraint.absolute_y_axis_tolerance = 1e-3
            orientation_constraint.absolute_z_axis_tolerance = 1e-3
            orientation_constraint.weight = 1.0

            goal_constraints.orientation_constraints.append(orientation_constraint)

        # Create Motion Plan Request
        req = MotionPlanRequest()
        req.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0
        req.workspace_parameters.header.frame_id = 'panda_link0'
        req.start_state.joint_state = start_state
        req.goal_constraints = [goal_constraints]
        req.pipeline_id = 'move_group'
        req.group_name = 'panda_manipulator'
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
        Computes the inverse kinematic for a given end effector pose

        Args
        ----
            pose (geometry_msgs/Pose): The goal pose for the end effector

        Returns
        -------
           A sensor_msgs/JointState object

        """
        # Build the IK Request
        get_pos_ik_req = self.build_ik_request(pose)

        # Call the IK service
        ik_response = await self.ik_client.call_async(get_pos_ik_req)

        return ik_response
    
    async def generate_motion_plan(self, request, response):
        """
        Generates a motion plan for the robot.

        Args
        ----
            request (PlanRequest): A plan request

        Returns
        -------
           An EmptyResponse object

        """
        # If start_pose is provided, compute the JointState for that pose
        if request.use_start_pose == True:

            ik_response = await self.compute_ik(request.start_pose)

            self.get_logger().info(str(ik_response))

            # If no IK solution is found
            if ik_response.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
                self.get_logger().warn("No IK solution found for start pose. Starting at current robot configuration instead.")
                start_joint_state = self.joint_state_msg

            # If IK solution is found, use that JointState as starting JointState
            else:
                start_joint_state = ik_response.solution.joint_state

        # Otherwise use the current JointState
        else:
            start_joint_state = self.joint_state_msg

        # Create the MoveGroup Goal
        plan_req = self.build_plan_request(start_joint_state, request.goal_pose, request.plan_mode, request.plan_only)

        # Call the move action client
        self.future_response = await self.plan_client.send_goal_async(plan_req)
        self.plan_response = await self.future_response.get_result_async()

        return response
    
    def joint_state_callback(self, msg):
        """
        Updated the current JointState of the robot.

        Args
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
