import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_interfaces.srv import PoseSrv, PlaceBox, Plan
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.msg import PositionIKRequest, CollisionObject, PlanningScene, PlanningSceneComponents, MotionPlanRequest, Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
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

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.plan_client = ActionClient(self, MoveGroup, "move_action",)

        self.joint_state_sub = self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.joint_state_msg = None

        self.plan = self.create_service(Plan, "plan", self.plan_callback, callback_group=self.cbgroup)
        self.ik_client = self.create_client(GetPositionIK, "compute_ik", callback_group=self.cbgroup)

        # Box publisher, service, and client
        self.box_publisher = self.create_publisher(PlanningScene, "planning_scene", 10)
        self.place_box = self.create_service(PlaceBox, "place_box", self.place_box_callback, callback_group=self.cbgroup)
        self.scene_client = self.create_client(GetPlanningScene, "get_planning_scene")

    def timer_callback(self):
        """
        Set up the transform listener to get the end effector position information.
        """
        try:
            self.base_ee_tf = self.tf_buffer.lookup_transform('panda_link0', 'panda_hand_tcp', rclpy.time.Time())
        
            self.get_logger().info("Panda TF: RECEIVED",once=True)
        except:
            self.get_logger().warn("Panda TF: WAITING")

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
    
    def get_ik_rqst_msg(self, pose):

        ikmsg = PositionIKRequest()
        ikmsg.group_name = 'panda_manipulator'
        ikmsg.robot_state.joint_state = self.joint_state_msg
        ikmsg.pose_stamped.header.frame_id = 'panda_link0'
        ikmsg.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ikmsg.pose_stamped.pose.position.x = pose.position.x
        ikmsg.pose_stamped.pose.position.y = pose.position.y
        ikmsg.pose_stamped.pose.position.z = pose.position.z
        ikmsg.pose_stamped.pose.orientation.x = pose.orientation.x
        ikmsg.pose_stamped.pose.orientation.y = pose.orientation.y
        ikmsg.pose_stamped.pose.orientation.z = pose.orientation.z
        ikmsg.pose_stamped.pose.orientation.w = pose.orientation.w
        ikmsg.timeout.sec = 5

        return ikmsg
    
    def get_request(self, start_state, goal_pose):
        """
        start: JointState
        goal: Pose
        """

        # Create constraints
        goal_constraints = Constraints()

        # for i in range(len(goal_state.name)):
        #     joint_constraint = JointConstraint()
        #     joint_constraint.joint_name = goal_state.name[i]
        #     joint_constraint.position = goal_state.position[i]
        #     joint_constraint.tolerance_above = 1e-3
        #     joint_constraint.tolerance_below = 1e-3
        #     joint_constraint.weight = 1.0

        #     goal_constraints.joint_constraints.append(joint_constraint)

        BV = BoundingVolume()

        solid_primitive = SolidPrimitive()
        solid_primitive.type = 1
        solid_primitive.dimensions = [2e-3, 2e-3, 2e-3]

        pose = Pose()
        pose.position.x = goal_pose.position.x
        pose.position.y = goal_pose.position.y
        pose.position.z = goal_pose.position.z

        BV.primitives =  [solid_primitive]
        BV.primitive_poses = [pose]

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'panda_link0'
        position_constraint.link_name = 'panda_link8'
        position_constraint.constraint_region = BV
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'panda_link0'
        orientation_constraint.link_name = 'panda_link8'
        orientation_constraint.orientation = goal_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 1e-3
        orientation_constraint.absolute_y_axis_tolerance = 1e-3
        orientation_constraint.absolute_z_axis_tolerance = 1e-3
        orientation_constraint.weight = 1.0

        goal_constraints.position_constraints.append(position_constraint)
        goal_constraints.orientation_constraints.append(orientation_constraint)

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

        self.get_logger().info(str(req))

        plan_request = MoveGroup.Goal()
        plan_request.request = req
        plan_request.planning_options.plan_only = True

        return plan_request
    
    async def compute_ik(self, pose):

        msg = self.get_ik_rqst_msg(pose)

        self.ik_response = await self.ik_client.call_async(GetPositionIK.Request(ik_request=msg))

        #self.get_logger().info(str(self.ik_response.error_code))

        return self.ik_response.solution.joint_state
    
    async def plan_callback(self, request, response):
        
        # If start_pose is provided
        if request.use_start_pose == True:

            start_joint_state = await self.compute_ik(request.start_pose)

        else:

            start_joint_state = self.joint_state_msg

        # self.get_logger().info(str(start_joint_state))

        # goal_joint_state = await self.compute_ik(request.goal_pose)
        # goal_joint_state = self.joint_state_msg
        # goal_joint_state.position[2] += 0.5
        
        # plan_msg = self.get_request(start_joint_state, goal_joint_state)
        plan_msg = self.get_request(start_joint_state, request.goal_pose)

        self.future_response = await self.plan_client.send_goal_async(plan_msg)
        self.plan_response = await self.future_response.get_result_async()

        return response
    
    def joint_state_callback(self, msg):
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
