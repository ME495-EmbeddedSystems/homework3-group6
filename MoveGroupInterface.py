import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from moveit_msgs.msg import PlannerParams
from moveit_msgs.srv import GetPlannerParams, SetPlannerParams, GraspPlanning, QueryPlannerInterface, GetCartesianPath
from moveit_msgs.action import MoveGroup, ExecuteTrajectory

# moveit.picknik.ai/main/html/move__group__interface_8cpp_source.html
"""
/planning_scene
/planning_scene_world
/recognized_object_array
/robot_description
/robot_description_semantic
/attached_collision_object
/clicked_point
/collision_object
/diagnostics
/display_contacts
/display_planned_path
/dynamic_joint_states
/franka/joint_states
/goal_pose
/initialpose
/trajectory_execution_event

Services
/apply_planning_scene
/check_state_validity
/compute_cartesian_path
/compute_fk
/compute_ik


"""


class MoveGroupInterface(Node, options, tf_buffer, wait_for_servers):

    def __init__(self):
        self.opt_ = self.options
        self.node_ = Node
        self.tf_buffer_ = tf_buffer
        self.wait_for_servers_ = wait_for_servers
        self.namespace_ = opt.namespace

        self.logger_ = self.node.get_logger()
        self.cb_group_ = MutuallyExclusiveCallbackGroup()
        
        # Available options, can redo / ignore some
        # opt.group_name 
        # opt.robot_model
        # opt.robot_description
        # opt.namespace

        self.can_look_ = False
        self.look_around_attempts_ = 0
        self.can_replan_ = False
        self.replan_delay_ = 2.0
        self.replan_attempts_ = 1
        self.goal_joint_tolerance_ = 1e-4
        self.goal_position_tolerance_ = 1e-4
        self.goal_orientation_tolerance_ = 1e-3
        self.allowed_planning_time_ = 5.0
        self.num_planning_attemps_ = 1
        
        self.velocity_scaling_factor_ = self.node_.get_parameter("velocity_scaling_factor").get_parameter_value().double_value
        self.acceleration_scaling_factor_ = self.node_.get_parameter("acceleration_scaling_factor").get_parameter_value().double_value

        self.initalizing_constraints_ = False

        self.move_action_client_ = ActionClient(self.node_, MoveGroup, self.namespace_ + "/move_group", self.cb_group_)
        if not self.move_action_client_.wait_for_service(timeout_sec=self.wait_for_servers_):
            raise RuntimeError("Timeout waiting for move_group action to become available")

        self.execute_action_client_ = ActionClient(self.node_, ExecuteTrajectory, self.namespace_ + "/execute_trajectory", self.cb_group_)
        if not self.execute_action_client_.wait_for_service(timeout_sec=self.wait_for_servers_):
            raise RuntimeError("Timeout waiting for execute_trajectory action to become available")
        
        self.query_service_ = self.node_.create_client(QueryPlannerInterface, self.namespace_ + "/query_planner_interface", self.cb_group_)
        if not self.query_service_.wait_for_service(timeout_sec=self.wait_for_servers_)
            raise RuntimeError("Timeout waiting for query_planner_interface service")

        self.get_params_service_ = self.node_.create_client(GetPlannerParams, self.namespace_ + "/get_planner_params", self.cb_group_)
        if not self.get_params_service_.wait_for_service(timeout_sec=self.wait_for_servers_)
            raise RuntimeError("Timeout waiting for get_planner_params service")

        self.set_params_service_ = self.node_.create_client(SetPlannerParams, self.namespace_ + "/set_planner_params", self.cb_group_)
        if not self.set_params_service_.wait_for_service(timeout_sec=self.wait_for_servers_)
            raise RuntimeError("Timeout waiting for set_planner_params service")

        self.cartesian_path_service_ = self.node_.create_client(GetCartesianPath, self.namespace_ + "/compute_cartesian_path", self.cb_group_)
        if not self.cartesian_path_service_.wait_for_service(timeout_sec=self.wait_for_servers_)
            raise RuntimeError("Timeout waiting for compute_cartesian_path service")

    def setPlannerParams(self, planner_id, group, params, replace=False):
        request = SetPlannerParams()
        request.planner_config = planner_id
        request.group = group
        request.params = params

        self.set_params_service_.call_async(request)
    
    def getPlannerParams(self, planner_id, group):
        request = GetPlannerParams()
        request.planner_config = planner_id
        request.group = group

        future_gpp_response = self.get_params_service_.call_async(request)

        if(future_gpp_response.valid()):
            return future_gpp_response.get()
    
    def setCanLook(self, n):
        if isinstance(n, bool):
            self.can_look_ = n
        else:
            self.logger_.error("Attempt to set can_look to invalid type")
    def getCanLook(self):
        return can_look_
    
    def setLookAroundAttemps(self, n):
        if isinstance(n, int):
            if(n < 0):
                self.logger_.error("Attempt to set look attempts negative. Set as positive instead")
            self.look_around_attempts_ = abs(n)
        else:
            self.logger_.error("Attempt to set look_around_attempts to invalid type")
    def getLookAroundAttemps(self):
        return self.look_around_attempts_
    
    def setCanReplan(self, n):
        if isinstance(n, bool):
            self.can_replan_ = n
        else:
            self.logger_.error("Attempt to set can_replan to invalid type")
    def getCanReplan(self):
        return self.can_replan_
    
    def setReplanDelay(self, n):
        if isinstance(n, int) or isinstance(n, float):
            self.replan_delay_ = float(n)
        else:
            self.logger_.error("Attempt to set replan_delay to invalid type")
    def getReplanDelay(self):
        return replan_delay_
    
    def setReplanAttempts(self, n):
        if isinstance(n, int):
            self.replan_attempts_ = n
        else:
            self.logger_.error("Attempt to set replan_attempts to invalid type")
    def getReplanAttempt(self):
        return replan_attempts_
    
    def setGoalJointTolerance(self, n):
        if isinstance(n, int) or isinstance(n, float):
            self.goal_joint_tolerance_ = float(n)
        else:
            self.logger_.error("Attempt to set goal_joint_tolerance to invalid type")
    def getGoalJointTolerance(self):
        return self.goal_joint_tolerance_
    
    def setGoalPositionTolerance(self, n):
        if isinstance(n, int) or isinstance(n, float):
            self.goal_position_tolerance_ = float(n)
        else:
            self.logger_.error("Attempt to set goal_position_tolerance to invalid type")
    def getGoalPositionTolerance(self):
        return self.goal_position_tolerance_
    
    def setGoalOrientationolerance(self, n):
        if isinstance(n, int) or isinstance(n, float):
            self.goal_orientation_tolerance_ = float(n)
        else:
            self.logger_.error("Attempt to set goal_orientation_tolreance to invalid type")
    def getGoalOrientationolerance(self):
        return self.goal_orientation_tolerance_
    
    def setAllowedPlanningTime(self, n):
        if isinstance(n, int) or isinstance(n, float):
            if(float(n) > 0.0):
                self.allowed_planning_time_ = float(n)
            else:
                self.logger_.error("Attempt to set allowed_planning_time to impossible time")
        else:
            self.logger_.error("Attempt to set allowed_planning_time to invalid type")
    def getAllowedPlanningTime(self):
        return self.allowed_planning_time_
    
    def setNumPlanningAttemps(self, n):
        if isinstance(n, int):
            self.num_planning_attemps_ = n
        else:
            self.logger_.error("Attempt to set num_planning_attempts to invalid type")
    def getNumPlanningAttemps(self):
        return self.num_planning_attemps_
    



