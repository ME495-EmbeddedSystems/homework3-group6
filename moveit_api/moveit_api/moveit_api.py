import rclpy
from rclpy.node import Node
from moveit_interfaces.srv import PoseSrv, PlaceBox
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PositionIKRequest, CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPositionIK, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from rclpy.callback_groups import ReentrantCallbackGroup


class MoveitAPI(Node):

    def __init__(self):
        super().__init__('moveit_api')

        self.cbgroup = ReentrantCallbackGroup()

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.initial_pose = Pose()
        self.goal_pose = Pose()
        self.req = PositionIKRequest()

        # Create Services
        self.set_initial = self.create_service(PoseSrv, 'set_initial', self.initial_pose_callback)
        self.set_goal = self.create_service(Empty, 'set_goal', self.goal_pose_callback)

        # Create Service Clients
        self.pos_ik_client = self.create_client(GetPositionIK, "compute_ik")

        # Box
        self.box_publisher = self.create_publisher(PlanningScene, "planning_scene", 10)
        self.place_box = self.create_service(PlaceBox, "place_box", self.place_box_callback, callback_group=self.cbgroup)
        self.scene_client = self.create_client(GetPlanningScene, "get_planning_scene")

    def timer_callback(self):
        pass

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
    
    
    def initial_pose_callback(self, request, response):
        
        self.initial_pose = request.pose

        self.start_IK_Callback()

        return response
    
    def goal_pose_callback(self, request, response):
        
        self.goal_pose = request.pose

        self.get_logger().info(str(self.initial_pose.position.x))

        return response
    
    def start_IK_Callback(self):
        """
        Populate desired starting configuration.

        This function send updated request with desired starting configuration and
        calls the compute_ik service.

        Returns
        -------
            None

        """

        # Populate IK Request
        self.req.group_name = 'panda_manipulator'
        self.req.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.req.robot_state.joint_state.header.frame_id = 'panda_link0'
        self.req.robot_state.joint_state.name = ['panda_joint1', 'panda_joint2', 'panda_joint3',
                                                'panda_joint4', 'panda_joint5', 'panda_joint6',
                                                'panda_joint7']
        self.req.robot_state.joint_state.position = self.initial_joint_states
        self.req.robot_state.multi_dof_joint_state.header.stamp = self.get_clock().now().to_msg()
        self.req.robot_state.multi_dof_joint_state.header.frame_id = 'panda_link0'
        self.req.robot_state.multi_dof_joint_state.joint_names = ['panda_joint1', 'panda_joint2',
                                                                 'panda_joint3', 'panda_joint4',
                                                                 'panda_joint5', 'panda_joint6',
                                                                 'panda_joint7']
        self.req.robot_state.is_diff = False
        self.req.avoid_collisions = True
        self.req.ik_link_name = 'panda_link8'
        self.req.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        self.req.pose_stamped.header.frame_id = 'panda_link0'
        self.req.pose_stamped.pose.position.x = self.initial_pose.position.x
        self.req.pose_stamped.pose.position.y = self.initial_pose.position.y
        self.req.pose_stamped.pose.position.z = self.initial_pose.position.z
        self.req.pose_stamped.pose.orientation.x = self.initial_pose.orientation.x
        self.req.pose_stamped.pose.orientation.y = self.initial_pose.orientation.y
        self.req.pose_stamped.pose.orientation.z = self.initial_pose.orientation.z
        self.req.pose_stamped.pose.orientation.w = self.initial_pose.orientation.w
        self.req.ik_link_names = ['panda_hand', 'panda_hand_tcp', 'panda_leftfinger', 'panda_link0',
                                 'panda_link1', 'panda_link2', 'panda_link3', 'panda_link4',
                                 'panda_link5', 'panda_link6', 'panda_link7', 'panda_link8',
                                 'panda_rightfinger']
        self.req.pose_stamped_vector = []
        self.req.timeout.sec = 60

        # Call Compute_IK
        self.future_start_IK = self.pos_ik_client.call_async(GetPositionIK.Request(ik_request=self.req))
        
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
