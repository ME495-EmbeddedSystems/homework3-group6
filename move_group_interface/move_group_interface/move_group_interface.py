import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

# https://wiki.ros.org/moveit_msgs
from moveit_msgs.msg import PlannerParams, PlanningScene, MotionPlanRequest, WorkspaceParameters, Constraints, RobotState, PositionConstraint, OrientationConstraint, JointConstraint, BoundingVolume, PlanningOptions, RobotTrajectory, PlanningSceneComponents, CollisionObject, PositionIKRequest
from moveit_msgs.srv import GetPlannerParams, SetPlannerParams, GraspPlanning, QueryPlannerInterfaces, GetCartesianPath, GetPlanningScene, GetPositionIK, GetPositionFK
from moveit_msgs.action import MoveGroup, ExecuteTrajectory, Pickup
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus

# https://github.com/ros-planning/moveit2/blob/main/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h
# https://github.com/ros-planning/moveit2/blob/main/moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp
# https://docs.ros.org/en/noetic/api/moveit_ros_planning_interface/html/move__group__interface_8cpp_source.html#l00407

"""
/planning_scene -- Check
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


class RobotModel():

    def __init__(self, group_name, joint_names, default_end_effector=None, default_base_link=None, robot_namespace="", desc="robot_description"):
        self.group_name = group_name
        self.joint_names = joint_names
        self.default_end_effector = default_end_effector
        self.default_base_link = default_base_link
        self.robot_namespace = robot_namespace
        self.robot_description = desc

class MoveGroupInterface():

    def __init__(self, node, robotModel, tf_buffer=None, namespace="", wait_for_servers=3.0):
        self.group_name_ = robotModel.group_name
        self.robot_description_ = robotModel.robot_description #unused
        self.joint_names_ = robotModel.joint_names
        self.end_effector_link_ = robotModel.default_end_effector
        self.base_link_ = robotModel.default_base_link
        self.robot_namespace_ = robotModel.robot_namespace

        self.namespace_ = namespace #Nice option but also unused 
        self.node_ = node
        self.tf_buffer_ = tf_buffer # Unsued? Why is it in the c++ version?
        self.wait_for_servers_ = wait_for_servers
        
        self.logger_ = self.node_.get_logger()
        self.clock_ = self.node_.get_clock()
        self.cb_group_ = MutuallyExclusiveCallbackGroup() # Change to MECB probablly 


        self.planning_pipeline_id_ = ""
        self.planner_id_ = ""
        self.workspace_parameters_ = None

        self.can_look_ = False
        self.look_around_attempts_ = 0
        self.can_replan_ = False
        self.replan_delay_ = 2.0
        self.replan_attempts_ = 10
        self.goal_joint_tolerance_ = 1e-4
        self.goal_position_tolerance_ = 1e-4
        self.goal_orientation_tolerance_ = 1e-3
        self.allowed_planning_time_ = 5.0
        self.num_planning_attemps_ = 1

        self.current_state_ = None
        self.start_state_ = None

        # !!! Add setters / getters for this, check if paramters exist else default to value
        self.max_velocity_scaling_factor_ = 0.1 #self.node_.get_parameter("velocity_scaling_factor").get_parameter_value().double_value
        self.max_acceleration_scaling_factor_ = 0.1 #self.node_.get_parameter("acceleration_scaling_factor").get_parameter_value().double_value

        self.initalizing_constraints_ = False

        self.constraints_ = Constraints()

        self.position_constraints_ = []
        self.orientation_constraints_ = []
        self.joint_constraints_ = []
        self.visibility_constraints_ = [] # Add funcitonality for increasd quality submisison -- doesnt exist rn

        self.joint_state_sub_ = self.node_.create_subscription(JointState, self.robot_namespace_+"joint_states", self.joint_state_callback, 10)

        self.planning_scene_pub_ = self.node_.create_publisher(PlanningScene, self.namespace_+"planning_scene", 10)

        self.move_action_client_ = ActionClient(self.node_, MoveGroup, self.namespace_ + "move_action", callback_group=self.cb_group_)
        if not self.move_action_client_.wait_for_server(timeout_sec=self.wait_for_servers_):
            raise RuntimeError("Timeout waiting for move_group action to become available")

        self.execute_action_client_ = ActionClient(self.node_, ExecuteTrajectory, self.namespace_ + "execute_trajectory", callback_group=self.cb_group_)
        if not self.execute_action_client_.wait_for_server(timeout_sec=self.wait_for_servers_):
            raise RuntimeError("Timeout waiting for execute_trajectory action to become available")
        
        self.query_service_ = self.node_.create_client(QueryPlannerInterfaces, self.namespace_ + "query_planner_interface", callback_group=self.cb_group_)
        if not self.query_service_.wait_for_service(timeout_sec=self.wait_for_servers_):
            raise RuntimeError("Timeout waiting for query_planner_interface service")

        self.get_params_service_ = self.node_.create_client(GetPlannerParams, self.namespace_ + "get_planner_params", callback_group=self.cb_group_)
        if not self.get_params_service_.wait_for_service(timeout_sec=self.wait_for_servers_):
            raise RuntimeError("Timeout waiting for get_planner_params service")

        self.set_params_service_ = self.node_.create_client(SetPlannerParams, self.namespace_ + "set_planner_params", callback_group=self.cb_group_)
        if not self.set_params_service_.wait_for_service(timeout_sec=self.wait_for_servers_):
            raise RuntimeError("Timeout waiting for set_planner_params service")

        self.cartesian_path_service_ = self.node_.create_client(GetCartesianPath, self.namespace_ + "compute_cartesian_path", callback_group=self.cb_group_)
        if not self.cartesian_path_service_.wait_for_service(timeout_sec=self.wait_for_servers_):
            raise RuntimeError("Timeout waiting for compute_cartesian_path service")
        
        self.planning_scene_service_ = self.node_.create_client(GetPlanningScene, self.namespace_+"get_planning_scene", callback_group=self.cb_group_)
        if not self.planning_scene_service_.wait_for_service(timeout_sec=self.wait_for_servers_):
            raise RuntimeError("Timeout waiting for get_planning_scene service")
        
        self.compute_ik_service_ = self.node_.create_client(GetPositionIK, self.namespace_+"compute_ik", callback_group=self.cb_group_)
        if not self.compute_ik_service_.wait_for_service(timeout_sec=self.wait_for_servers_):
            raise RuntimeError("Timeout waiting for compute_ik service")

        self.compute_fk_service_ = self.node_.create_client(GetPositionFK, self.namespace_+"compute_fk", callback_group=self.cb_group_)
        if not self.compute_fk_service_.wait_for_service(timeout_sec=self.wait_for_servers_):
            raise RuntimeError("Timeout waiting for compute_fk service")



    """

    Setters / Getters 
    !!! (Sort them? Alpha?)

    """

    def setPlannerParams(self, planner_id, group, params, replace=False):
        request = SetPlannerParams()
        request.planner_config = planner_id
        request.group = group
        request.params = params

        self.set_params_service_.call_async(request)
    
    async def getPlannerParams(self, planner_id, group):
        request = GetPlannerParams()
        request.planner_config = planner_id
        request.group = group

        response = await self.get_params_service_.call_async(request)
        return response

    def setPlanningPipelineID(self, pipeline_id):
        if isinstance(pipeline_id, str):
            if(pipeline_id != self.planning_pipeline_id_):
                self.planning_pipeline_id_ = pipeline_id
                self.planner_id_ = ""
        else:
            self.logger_.error("Attempt to set planning_pipeline_id to invalid type")

    def getPlanningPipelineID(self):
        return self.planning_pipeline_id_
    
    def getDefaultPlannerID(self):
        # !!! Fix this. Possible in Ros2 but more annoying
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
        # !!! Fix this
        self.start_state_ = RobotState()
        self.start_state_.joint_state = joint_values
        if(not(mdof_joint_values is None)):
            self.start_state_ .multi_dof_joint_state = mdof_joint_values
        if(not(attached_objects is None)):
            self.start_state_ .attached_collision_objects = attached_objects
        if(not(is_diff is None)):
            self.start_state_ .is_diff = is_diff

    def setWorkspaceParamaters(self, minx, maxx, miny, maxy, minz, maxz, frame=None):
        if((isinstance(minx, int) or isinstance(minx, float) ) and
           (isinstance(miny, int) or isinstance(miny, float) ) and
           (isinstance(minz, int) or isinstance(minz, float) ) and
           (isinstance(maxx, int) or isinstance(maxx, float) ) and
           (isinstance(maxy, int) or isinstance(maxy, float) ) and
           (isinstance(maxz, int) or isinstance(maxz, float) )):

            if(frame is None):
                frame = self.base_link_

            self.workspace_parameters_ = WorkspaceParameters()
            self.workspace_parameters_.header.frame_id = frame
            self.workspace_parameters_.header.stamp = self.clock_.now().to_msg()

            self.workspace_parameters_.min_corner.x = minx
            self.workspace_parameters_.min_corner.y = miny
            self.workspace_parameters_.min_corner.z = minz

            self.workspace_parameters_.max_corner.x = maxx
            self.workspace_parameters_.max_corner.y = maxy
            self.workspace_parameters_.max_corner.z = maxz
        else:
            self.logger_().error("Attempt to set workspace parameter coordiante to invalid type")
    
    """

    Constraint Functions

    """

    def addPositionConstraint(self, pose_stamped, link=None, offset=None, tolerance=None, weight=1.0):
        new_pc = PositionConstraint()
        new_pc.header = pose_stamped.header

        if(not(offset is None)):
            new_pc.target_point_offset.x = offset.x
            new_pc.target_point_offset.y = offset.y
            new_pc.target_point_offset.z = offset.z

        if(link is None):
            new_pc.link_name = self.end_effector_link_
        else:
            new_pc.link_name = link
        
        if(tolerance is None):
            tolerance = self.goal_position_tolerance_

        BV = BoundingVolume()
        
        SP = SolidPrimitive()
        SP.type = 2
        SP.dimensions = [tolerance]

        BV.primitives =  [SP]
        BV.primitive_poses = [pose_stamped.pose]

        new_pc.constraint_region = BV
        new_pc.weight = weight

        self.position_constraints_.append(new_pc)

    def clearPositionConstraints(self):
        self.position_constraints_ = []

    def addOrientationConstraint(self, pose_stamped, link=None, tolerance=None, weight=1.0):
        new_oc = OrientationConstraint()
        new_oc.header = pose_stamped.header
        new_oc.orientation = pose_stamped.pose.orientation
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
        self.orientation_constraints_ = []
    
    def addPoseConstraint(self, pose_stamped, link=None, lin_tol=None, ang_tol=None, weight=1.0):
        self.addOrientationConstraint(pose_stamped, link=link, tolerance=ang_tol, weight=weight)
        self.addPositionConstraint(pose_stamped, link=link, tolerance=lin_tol, weight=weight)

    def clearPoseConstraints(self):
        self.clearOrientationConstraints()
        self.clearPositionConstraints()
    
    def addJointConstraints(self, joint_states, tol_a_array=None, tol_b_array=None, weight_array=None):
        for i in range(len(joint_states.name)):
            if(tol_a_array is None):
                tol_a = None
            else:
                tol_a = tol_a_array[i]

            if(tol_b_array is None):
                tol_b = None
            else:
                tol_b = tol_b_array[i]

            if(weight_array is None):
                weight = 1.0
            else:
                weight = weigh_array[i]

            self.addJointConstraint(joint_states.name[i], joint_states.position[i], tol_a=tol_a, tol_b=tol_b, weight=weight)
    
    def addJointConstraint(self, joint_name, position, tol_a=None, tol_b=None, weight=1.0):
        new_jc = JointConstraint()

        if(tol_a is None):
            tol_a = self.goal_joint_tolerance_
        if(tol_b is None):
            tol_b = self.goal_joint_tolerance_
            
        new_jc.joint_name = joint_name
        new_jc.position = position
        new_jc.tolerance_above = tol_a
        new_jc.tolerance_below = tol_b
        new_jc.weight = weight

        self.joint_constraints_.append(new_jc)

    def clearJointConstraints(self):
        self.joint_constraints_ = []

    def clearAllConstraints(self):
        self.clearPoseConstraints()
        self.clearJointConstraints()  

    def mergeJointConstraints(self):
        # !!! Fix this: Validate, double check logic
        # For sure doesnt work now if many constraints with same joint_name
        # Possible fix: find all indicies with same joint name
        # Merged back to back with those
        # Separate merge function
        # Seems doable

        merged_jc = []

        for i in range(len(self.joint_constraints_)):
            add = True
            for j in range(i+1,len(self.joint_constraints_)):
                if(self.joint_constraints_[i].joint_name == self.joint_constraints_[j].joint_name):
                    add = False
                    JC = JointConstraint()
                    first_jc = self.joint_constraints_[i]
                    second_jc = self.joint_constraints_[j]
                    low = max(first_jc.position - first_jc.tolerance_below, second_jc.position - second_jc.tolerance_below)
                    high = min(first_jc.position + first_jc.tolerance_above, second_jc.position + second_jc.tolerance_above)
                    if(low > high):
                        self.logger_.error("Incompatiable Joint Constraints for joint " + first_jc.joint_name + ", discarding constraint")
                    else:
                        JC.joint_name = first_jc.joint_name
                        JC.position = max(low, min((first_jc.position * first_jc.weight + second_jc.position * second_jc.weight)/(first_jc.weight + second_jc.weight)), high)
                        JC.weight = (first_jc.weight + second_jc.weight)/2.0
                        JC.tolerance_above = max(0.0, high - JC.position)
                        JC.tolerance_below = max(0.0, JC.position - low)
                        merged_jc.append(JC)
                    break
            if(add):
                merged_jc.append(self.joint_constraints_[i])
        
        self.joint_constraints_ = merged_jc
    
    """

    Action functions

    """

    def constructPlannerOptions(self, plan_only=True):
        options = PlanningOptions()
        options.plan_only = plan_only
        options.look_around = self.can_look_
        options.look_around_attempts = self.look_around_attempts_
        options.replan = self.can_replan_
        options.replan_attempts = self.replan_attempts_
        options.replan_delay = self.replan_delay_
        return options

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
        request.start_state = self.start_state_
        self.constraints_.position_constraints = self.position_constraints_
        self.constraints_.orientation_constraints = self.orientation_constraints_
        self.constraints_.joint_constraints = self.joint_constraints_

        # !!! Fix this function 
        #self.constraints_ = self.mergeJointConstraints()
        request.goal_constraints = [self.constraints_]
        return request
    

    async def makeMotionPlanRequest(self, plan_only=True, start_state = None, eef_pose_stamped=None, clean=True):
        if(not(eef_pose_stamped is None)):
            self.addPositionConstraint(eef_pose_stamped)
            self.addOrientationConstraint(eef_pose_stamped)

        # If plan_only = False (executing trajectory right after), MUST start at current state
        if((start_state is None and self.start_state_ is None) or not(plan_only)):
            self.setStartState(self.current_state_)
        elif(not(start_state is None)):
            self.setStartState(start_state)
        
        goal = MoveGroup.Goal()
        goal.planning_options = self.constructPlannerOptions(plan_only)
        goal.request = self.constructMotionPlanRequest()
        goal_handle = await self.move_action_client_.send_goal_async(goal)

        if not goal_handle.accepted:
            self.logger_.info("Goal not accepted")
            return
        self.logger_.info("Goal accepted.")

        res = await goal_handle.get_result_async()
        result = res.result
        status = res.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.logger_.info("Goal succeeded!")
        else:
            self.logger_.info('Goal failed with status: {0}'.format(status))
        if(clean):
            self.cleanUp()
        return result, status
    
    async def executeTrajectory(self, trajectory):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        goal_handle = await self.execute_action_client_.send_goal_async(goal)

        if not goal_handle.accepted:
            self.logger_.info("Goal not accepted")
            return
        self.logger_.info("Goal accepted.")

        res = await goal_handle.get_result_async()
        result = res.result
        status = res.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.logger_.info("Goal succeeded!")
        else:
            self.logger_.info('Goal failed with status: {0}'.format(status))
        return result, status
    
    def joint_state_callback(self, msg):
        self.current_state_ = JointState()
        self.current_state_.header = msg.header

        for i in range(len(msg.name)):
            for j in self.joint_names_:
                if(msg.name[i] == j):
                    self.current_state_.name.append(msg.name[i])
                    self.current_state_.position.append(msg.position[i])
                    self.current_state_.velocity.append(msg.velocity[i])
                    self.current_state_.effort.append(msg.effort[i])

    async def getPlanningScene(self, components=PlanningSceneComponents()):
        planning_scene = await self.planning_scene_service_.call_async(GetPlanningScene.Request(components=components))
        return planning_scene.scene

    async def addCollisionObject(self, shape, pose_stamped):
        scene = await self.getPlanningScene()
        
        obj = CollisionObject()
        obj.header = pose_stamped.header
        obj.pose = pose_stamped.pose
        obj.primitives = [shape]
        obj.primitive_poses = [Pose()]

        scene.world.collision_objects.append(obj)

        self.planning_scene_pub_.publish(scene)

    def cleanUp(self):
        """Clean up after planning"""
        self.clearAllConstraints()
        self.start_state_ = None