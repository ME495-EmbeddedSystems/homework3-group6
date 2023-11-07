import rclpy
from rclpy.node import Node
from move_group_interface.move_group_interface import RobotModel
from move_group_interface.move_group_interface import MoveGroupInterface
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty


class move_group_test(Node):
    def __init__(self):
        super().__init__("move_group_test")

        joint_names = ['panda_joint1', 'panda_joint2',
                       'panda_joint3', 'panda_joint4',
                       'panda_joint5', 'panda_joint6',
                       'panda_joint7']
        group_name = 'panda_manipulator'
        end_effector_link = 'panda_link8'
        base_link = 'panda_link0'

        self.robotModel = RobotModel(group_name, joint_names,
                                     default_end_effector=end_effector_link,
                                     default_base_link=base_link)
        self.API = MoveGroupInterface(self, self.robotModel)
        self.API.setWorkspaceParamaters(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0)
        self.API.setPlanningPipelineID("move_group")

        self.box = SolidPrimitive()
        self.box.type = 1
        self.box.dimensions = [0.5, 0.5, 0.5]

        self.box_pose = PoseStamped()
        self.box_pose.header.frame_id = "panda_link0"
        self.box_pose.pose.position.x = 1.0
        self.box_pose.pose.position.y = 1.0
        self.box_pose.pose.position.z = 0.0

        poseGoal = PoseStamped()
        poseGoal.header.frame_id = "panda_link0"
        poseGoal.pose.position.x = 0.2
        poseGoal.pose.position.y = 0.2
        poseGoal.pose.position.z = 0.3
        self.API.addPositionConstraint(poseGoal)

        self.res = None
        self.joint_start = JointState()
        self.joint_start.name = joint_names

        self.joint_start.position = [0.0, -1.0, 0.0, -2.0, 0.0, 1.57, 1.0]

        # It is possible to set a different start state for planning!
        # self.API.setStartState(self.joint_start)
        self.traj = None

        self.place_box_service = self.create_service(Empty, "place_box",
                                                     self.place_box_callback)
        self.plan_trajectory_service =\
            self.create_service(Empty, "plan_trajectory",
                                self.plan_trajectory_callback)
        self.exec_trajectory_service =\
            self.create_service(Empty, "exec_trajectory",
                                self.exec_trajectory_callback)

        self.tmr = self.create_timer(0.01, self.timer_callback)

    async def timer_callback(self):
        self.tmr.cancel()
        self.get_logger().debug("Not crashing!")
        res, stat = await self.API.makeMotionPlanRequest()

    async def place_box_callback(self, request, response):
        await self.API.addCollisionObject(self.box, self.box_pose)
        return response

    async def plan_trajectory_callback(self, request, response):
        self.res, self.stat1 =\
            await self.API.makeMotionPlanRequest(plan_only=True)
        self.traj = self.res.planned_trajectory
        return response

    async def exec_trajectory_callback(self, request, response):
        self.res2, self.stat2 = await self.API.executeTrajectory(self.traj)
        return response


def main(args=None):
    """Run api node."""
    rclpy.init(args=args)
    myMoveGroupTest = move_group_test()
    rclpy.spin(myMoveGroupTest)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
