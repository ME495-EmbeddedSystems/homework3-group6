import rclpy
from rclpy.node import Node
from moveit_interfaces.srv import PoseSrv, PlaceBox, Plan
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.msg import PositionIKRequest, CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPositionIK, GetPlanningScene
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

        self.joint_state_sub = self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.joint_state_msg = None

        self.plan = self.create_service(Plan, "plan", self.plan_callback, callback_group=self.cbgroup)
        self.ik_client = self.create_client(GetPositionIK, "compute_ik", callback_group=self.cbgroup)

        # Box
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
    
    async def compute_ik(self, pose):

        msg = self.get_ik_rqst_msg(pose)

        self.ik_response = await self.ik_client.call_async(GetPositionIK.Request(ik_request=msg))

        self.get_logger().info(str(self.ik_response.error_code))

        return self.ik_response.solution.joint_state
    
    async def plan_callback(self, request, response):
        
        # If start_pose is provided
        if request.use_start_pose == True:

            start_joint_angles = await self.compute_ik(request.start_pose)

        else:

            start_joint_angles = self.joint_state_msg.position

        self.get_logger().info(str(start_joint_angles))

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
