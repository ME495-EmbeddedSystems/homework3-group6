import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# https://wiki.ros.org/moveit_msgs
from moveit_msgs.msg import PlannerParams, MotionPlanRequest, WorkplaceParameters, Constraints, RobotState, PositionConstraint, OrientationConstraint, JointConstraint
from moveit_msgs.srv import GetPlannerParams, SetPlannerParams, GraspPlanning, QueryPlannerInterface, GetCartesianPath
from moveit_msgs.action import MoveGroup, ExecuteTrajectory

from sensor_msgs.msg import JointState
from enum import Enum, auto

# https://github.com/ros-planning/moveit2/blob/main/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h
# https://github.com/ros-planning/moveit2/blob/main/moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp
# https://docs.ros.org/en/noetic/api/moveit_ros_planning_interface/html/move__group__interface_8cpp_source.html#l00407
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
class TargetState(Enum):
    JOINT = auto(),
    POSE = auto(),
    POSITION = auto(),
    ORIENTATION = auto()

class RobotModel():

    def __init__(self, group_name, joint_names, default_end_effector=None,desc="robot_description"):
        self.group_name_ = group_name
        self.joint_names = joint_names
        self.default_end_effector = default_end_effector
        self.robot_description = desc

class MoveGroupInterface():

    def __init__(self, node, robotModel, namespace, tf_buffer, wait_for_servers):
        self.group_name_ = robotModel.group_name
        self.robot_description_ = robotModelrobot_description
        self.end_effector_link_ = robotMode.default_end_effector
        self.namespace_ = self.namespace

        self.node_ = node
        self.tf_buffer_ = tf_buffer
        self.wait_for_servers_ = wait_for_servers
        
        self.logger_ = self.node_.get_logger()
        self.clock_ = self.node_.get_clock()
        self.cb_group_ = MutuallyExclusiveCallbackGroup()

        # Available options, can redo / ignore some
        # opt.group_name 
        # opt.robot_model
        # opt.robot_description
        # opt.namespace

        # self.pose_targets_
        # self.end_effector_link_
        # self.pose_reference_frame_
        # self.target_type_
        # self.start_state_

        self.planning_pipeline_id_ = ""
        self.planner_id_ = ""
        self.workspace_parameters_ = WorkplaceParameters()

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

        self.current_state_ = None
        
        self.max_velocity_scaling_factor_ = self.node_.get_parameter("velocity_scaling_factor").get_parameter_value().double_value
        self.max_acceleration_scaling_factor_ = self.node_.get_parameter("acceleration_scaling_factor").get_parameter_value().double_value

        self.initalizing_constraints_ = False

        self.constraints_ = Constraints()

        self.position_constraints_ = []
        self.orientation_constraints_ = []
        self.joint_constraints_ = []

        # Subscribe to /joint_states

        self.move_action_client_ = ActionClient(self.node_, MoveGroup, self.namespace_ + "/move_group", self.cb_group_)
        if not self.move_action_client_.wait_for_service(timeout_sec=self.wait_for_servers_):
            raise RuntimeError("Timeout waiting for move_group action to become available")

        self.execute_action_client_ = ActionClient(self.node_, ExecuteTrajectory, self.namespace_ + "/execute_trajectory", self.cb_group_)
        if not self.execute_action_client_.wait_for_service(timeout_sec=self.wait_for_servers_):
            raise RuntimeError("Timeout waiting for execute_trajectory action to become available")
        
        self.query_service_ = self.node_.create_client(QueryPlannerInterface, self.namespace_ + "/query_planner_interface", self.cb_group_)
        if not self.query_service_.wait_for_service(timeout_sec=self.wait_for_servers_):
            raise RuntimeError("Timeout waiting for query_planner_interface service")

        self.get_params_service_ = self.node_.create_client(GetPlannerParams, self.namespace_ + "/get_planner_params", self.cb_group_)
        if not self.get_params_service_.wait_for_service(timeout_sec=self.wait_for_servers_):
            raise RuntimeError("Timeout waiting for get_planner_params service")

        self.set_params_service_ = self.node_.create_client(SetPlannerParams, self.namespace_ + "/set_planner_params", self.cb_group_)
        if not self.set_params_service_.wait_for_service(timeout_sec=self.wait_for_servers_):
            raise RuntimeError("Timeout waiting for set_planner_params service")

        self.cartesian_path_service_ = self.node_.create_client(GetCartesianPath, self.namespace_ + "/compute_cartesian_path", self.cb_group_)
        if not self.cartesian_path_service_.wait_for_service(timeout_sec=self.wait_for_servers_):
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

        if(future_gpp_response.done()):
            return future_gpp_response.get()
    def setPlanningPipelineID(self, pipeline_id):
        if isinstance(pipeline_id, str):
            if(pipeline_id != planning_pipeline_id_):
                self.planning_pipeline_id_ = pipeline_id
                self.planner_id_ = ""
        else:
            self.logger_.error("Attempt to set planning_pipeline_id to invalid type")
    def getPlanningPipelineID(self):
        return self.planning_pipeline_id_
    
    def getDefaultPlannerID(self, group):
        default_planner_id = self.node_.get_parameter("move_group/default_planning_pipeline").get_parameter_value().string_value
        return default_planner_id

    def setPlannerId(self,n):
        if isinstance(n, str):
            self.planner_id_ = n
        else:
            self.logger_.error("Attempt to set planner_id to invalid type")
    
    def getPlannerId(self):
        return self.planner_id_

    def setCanLook(self, n):
        if isinstance(n, bool):
            self.can_look_ = n
        else:
            self.logger_.error("Attempt to set can_look to invalid type")
    def getCanLook(self):
        return self.can_look_
    
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
        return self.replan_delay_
    
    def setReplanAttempts(self, n):
        if isinstance(n, int):
            self.replan_attempts_ = n
        else:
            self.logger_.error("Attempt to set replan_attempts to invalid type")

    def getReplanAttempt(self):
        return self.replan_attempts_
    
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

    def setStartState(self, joint_values, mdof_joint_values=None, attached_objects=None, is_diff = None):
        start_state = RobotState()
        start_state.joint_states = joint_values
        if(not(mdof_joint_values is None)):
            start_state.multi_dof_joint_state = mdof_joint_values
        if(not(attached_objects is None)):
            start_state.attached_collision_objects = attached_objects
        if(not(is_diff is None)):
            start_state.is_diff = is_diff
        self.start_state_ = start_state

    def addPositionConstraint(self, pose_stamped, link=None, tolerance=None, weight=1.0):
        new_pc = PositionConstraint()
        new_pc.header = pose_stamped.header

        # This may cause an error because position is a point msg and offset is a vector3f
        new_pc.target_point_offset = pose_stamped.pose.position

        if(link is None):
            new_pc.link_name = self.default_end_effector
        else:
            new_pc.link_name = link
        
        if(tolerance is None):
            tolerance = self.goal_position_tolerance_

        BV = BoundingVolume()
        BV.type = 1
        BV.dimensions[0] = (2 * tolerance)
        BV.dimensions[1] = (2 * tolerance)
        BV.dimensions[2] = (2 * tolerance)

        new_pc.constraint_region = BV
        new_pc.weight = weight

        self.position_constraints_.append(new_pc)

    def clearPositionConstraints(self):
        self.position_constraints_ = None

    def addOrientationConstraint(self, pose_stamped, link=None, tolerance=None, weight = 1.0):
        new_oc = OrientationConstraint()
        new_oc.header = pose_stamped.header
        new_oc.orienation = pose_stamped.pose.orienation
        if(link is None):
            new_oc.link_name = self.end_effector_link_
        else:
            new_oc.link_name = link
        
        if(tolerance is None):
            new_oc.absolute_x_axis_tolerance = self.goal_orientation_tolerance_
            new_oc.absolute_y_axis_tolerance = self.goal_orientation_tolerance_
            new_oc.absolute_z_axis_tolerance = self.goal_orientation_tolerance_
        else:
            new_oc.absolute_x_axis_tolerance = tolerance
            new_oc.absolute_y_axis_tolerance = tolerance
            new_oc.absolute_z_axis_tolerance = tolerance
        
        new_oc.weight = weight

        self.orientation_constraints_.append(new_oc)

    def clearOrientationConstraints(self):
        self.orientation_constraints_ = None

    # Add joint_constraint functionality

    def constructMotionPlanRequest(self):
        request = MotionPlanRequest()
        request.group_name = self.group_name_
        request.num_planning_attempts = self.num_planning_attemps_
        request.max_velocity_scaling_factor = self.max_velocity_scaling_factor_
        request.max_acceleration_scaling_factor = self.max_acceleration_scaling_factor_
        request.allowed_planning_time = self.allowed_planning_time_
        request.pipeline_id = self.planning_pipeline_id_
        request.planner_id = self.planner_id_
        request.workspace_parameters = self.workspace_parameters_
        if(self.start_state_ is None):
            self.getCurrentState()
        request.start_state = self.start_state_

        self.constraints_.position_constraints = self.position_constraints_
        self.constraints_.orientation_constraints = self.orientation_constraints_
        
        self.constraints_ = self.mergeConstraints(self.constraints_, self.constraints_)
        request.goal_constraints = self.constraints_
        return request
    
    def mergeConstraints(self, first_c, second_c):

        merged_c = Constraints()

        for i in first_c.joint_constraints:
            add = True
            for j in second_c.joint_constraints:
                if(j.joint_name == i.joint_name):
                    add = False
                    JC = JointConstraint()
                    low = max(i.position - i.tolerance_below, j.position - j.tolerance_below)
                    high = min(i.position + i.tolerance_above, j.position + j.tolerance_above)
                    if(low > high):
                        self.logger_.error("Incompatiable Joint Constraints for joint " + i.joint_name + ", discarding constraint")
                    else:
                        JC.joint_name = i.joint_name
                        JC.position = max(low, min((i.position * i.weight + j.position * j.weight)/(i.weight + j.weight)), high)
                        JC.weight = (i.weight + j.weight)/2.0
                        JC.tolerance_above = max(0.0, high - JC.position)
                        JC.tolerance_below = max(0.0, JC.position - low)
                        merged_c.joint_constraints.push_back(JC)
                    break
                if(add):
                    merged_c.joint_constraints.push_back(i)

        for i in second_c.joint_constraints:
            add = True
            for j in first_c.joint_constraints:
            if (i.joint_name==j.joint_name):
                add = False
                break
            if (add):
                merged_c.joint_constraints.push_back(i)

        merged_c.position_constraints = first_c.position_constraints
        for i in second_c.position_constraints:
            merged_c.position_constraints.push_back(i)
        
        merged_c.orientation_constraints = first_c.orientation_constraints
        for i in second_c.orientation_constraints:
            merged_c.orientation_constraints.push_back(i)

        merged_c.visibility_constraints = first_c.visibility_constraints
        for i in second_c.visibility_constraints:
            merged_c.visibility_constraints.push_back(i)
        
        return(merged_c)

    def makeMotionPlanRequest(self, start_state = None, eef_pose_stamped=None):
        if(not(eef_pose_stamped is None)):
            self.addPositionConstraint(eef_pose_stamped)
            self.addOrientationConstraint(eef_pose_stamped)
        req = self.constructMotionPlanRequest()
        self.future_move = self.move_action_client_.call_async(req)
        self.start_state_ = None
        self.clearOrientationConstraints()
        self.clearPositionConstraints()
        if(self.future_move.done()):
            # Do something?
            response = self.future_move.get()
    
    def getCurrentState():
        # listen to /joint_states to get current state
        # only need states for joints we care about
        # check joint_names
        self.logger_.debug("Getting Current State!")

    
    def setWorkplaceParamaters(self, frame, minx, maxx, miny, maxy, minz, maxz):
        if((isinstance(minx, int) or isinstance(minx, float) ) and
           (isinstance(miny, int) or isinstance(miny, float) ) and
           (isinstance(minz, int) or isinstance(minz, float) ) and
           (isinstance(maxx, int) or isinstance(maxx, float) ) and
           (isinstance(maxy, int) or isinstance(maxy, float) ) and
           (isinstance(maxz, int) or isinstance(maxz, float) )
        ):
            self.workspace_parameters_.header.frame_id = frame
            self.workspace_parameters_.header.stamp = self.clock_.now()

            self.workspace_parameters_.min_corner.x = minx
            self.workspace_parameters_.min_corner.y = miny
            self.workspace_parameters_.min_corner.z = minz

            self.workspace_parameters_.max_corner.x = maxx
            self.workspace_parameters_.max_corner.y = maxy
            self.workspace_parameters_.max_corner.z = maxz
        else:
            self.logger_().error("Attempt to set workplace parameter coordiante to invalid type")
    



