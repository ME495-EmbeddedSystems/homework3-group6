import rclpy
from rclpy.node import Node

import move_group_interface
import move_group_interface.move_group_interface.RobotModel
from move_group_interface.move_group_interface import RobotModel
from move_group_interface.move_group_interface import MoveGroupInterface

from move_group_interface import RobotModel, MoveGroupInterface

from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive

from sensor_msgs.msg import JointState

class move_group_test(Node):
    def __init__(self):
        super().__init__("move_group_test")

        joint_names = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
        group_name = 'panda_manipulator'
        end_effector_link = 'panda_link8'
        base_link = 'panda_link0'

        self.robotModel = RobotModel(group_name, joint_names, default_end_effector=end_effector_link, default_base_link=base_link)
        self.API = MoveGroupInterface(self, self.robotModel)
        self.API.setWorkspaceParamaters(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0)
        self.API.setPlanningPipelineID("move_group")

        self.box = SolidPrimitive()
        self.box.type = 1
        self.box.dimensions = [0.5,0.5,0.5]

        self.box_pose = PoseStamped()
        self.box_pose.header.frame_id = "panda_link0"
        self.box_pose.pose.position.x = 1.0
        self.box_pose.pose.position.y = 1.0
        self.box_pose.pose.position.z = 0.0

        poseGoal = PoseStamped()
        poseGoal.header.frame_id = "panda_link0"
        poseGoal.pose.position.z = 0.2
        poseGoal.pose.position.x = 0.2
        self.API.addOrientationConstraint(poseGoal)
        self.API.addPositionConstraint(poseGoal)
        
        self.res = None
        self.traj = None
        
        tmr = self.create_timer(0.01, self.timer_callback)
        self.i = 0

    async def timer_callback(self):
        self.i +=1
        if(self.i == 50):
            await self.API.addCollisionObject(self.box, self.box_pose)

        if(self.i == 100):
            self.res, self.stat1 = await self.API.makeMotionPlanRequest(plan_only=True)
            self.traj = self.res.planned_trajectory
        
        if(not(self.traj is None) and self.i == 300):
            self.res2, self.stat2 = await self.API.executeTrajectory(self.traj)

def main(args=None):
    """Run api node."""
    rclpy.init(args=args)
    myMoveGroupTest= move_group_test()
    rclpy.spin(myMoveGroupTest)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
