import dataclasses
from typing import Optional
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlannerParams,
    PlanningOptions,
    PlanningScene,
    PlanningSceneComponents,
    PositionConstraint,
    RobotState,
    WorkspaceParameters,
    AttachedCollisionObject,
    MoveItErrorCodes,
)
from moveit_msgs.srv import (
    GetCartesianPath,
    GetPlannerParams,
    GetPlanningScene,
    GetPositionFK,
    GetPositionIK,
    QueryPlannerInterfaces,
    SetPlannerParams,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState, MultiDOFJointState
from shape_msgs.msg import SolidPrimitive


@dataclasses.dataclass
class RobotModel:
    """
    Model of an open-chain robot.

    necessary data of the robot for move group to plan with.

    group_name (str) : The name of the move_group contantaing the joints
    joint_names (str[]) : The list of joints in the move_group
    default_end _effector (str) : A default end effector link (optional)
    default_base_link (str) : A default base link (optional)
    robot_namespace (str) : Robot namespace affecting
        joint_states publishing (optional)
    robot_description (str) : A robot description topic (optional, unused)
    """

    group_name: str
    joint_names: list[str]
    # Note: Original code below two are optional,
    # I think they should be required.
    default_end_effector: str
    default_base_link: str
    robot_namespace: str = ""
    robot_description: str = "robot_description"


class MoveGroupInterface:
    """
    A motion planning interface that uses MoveIt!.

    PUBLISHES:
        planning_scene (moveit_msgs/msg/PlanningScene) - A specification of
            the robot, its state, and the objects it can collide with.

    SUBSCRIBES:
        jointstates (sensor_msgs/msg/JointState) - The (timestamped) joint
            positions of the robot.

    ACTION CLIENTS:
        execute_trajectory (moveit_msgs/action/ExecuteTrajectory) -
            Executes a joint-space trajectory.
        move_action (moveit_msgs/action/MoveGroup) -
            Plans / plans and executes the robot's trajectory.

    SERVICE CLIENTS:
        compute_cartesian_path (moveit_msgs/srv/GetCartesianPath) - Computes a
            joint-space trajectory, given end-effector waypoints.
        compute_fk (moveit_msgs/srv/GetPositionFK) -
            Computes Forward Kinematics.
        compute_ik (moveit_msgs/srv/GetPositionIK) -
            Computes Inverse Kinematics.
        get_planner_params (moveit_msgs/srv/GetPlannerParams) -
            Gets the parametrization of the planner corresponding to a planner
            config and planning group.
        get_planning_scene (moveit_msgs/srv/GetPlanningScene) -
            Gets parts of the planninng scene that are of interest.
        query_planner_interface (moveit_msgs/srv/QueryPlannerInterfaces) -
            Gives a description of the planner interface.
        set_planner_params (moveit_msgs/srv/SetPlannerParams) -
            Parameterizes the planner, and sets its config and group.

    """

    def __init__(
        self,
        node: Node,
        robotModel: RobotModel,
        tf_buffer=None,
        namespace: str = "",
        wait_for_servers_sec: float = 3.0,
    ):
        """
        Initialize the motion planning interface.

        Args:
        ----
            node (rclpy/Node) : Motion planning node.
            robotModel (RobotModel) : Model of the robot.
            tf_buffer (tf2_ros.buffer) : Transform buffer (optional, unused)
            namespace (str) : namespace of the move_group node.
                Used to identify actions and services.
            wait_for_servers_sec (float) : Time to wait for servers (seconds).
                Defaults to 3.0 seconds

        """
        self.group_name_ = robotModel.group_name
        self.joint_names_ = robotModel.joint_names
        self.end_effector_link_ = robotModel.default_end_effector
        self.base_link_ = robotModel.default_base_link
        self.robot_namespace_ = robotModel.robot_namespace
        self.robot_description_ = robotModel.robot_description  # unused

        self.node_ = node
        self.tf_buffer_ = tf_buffer  # Unsued? Why is it in the c++ version?
        self.namespace_ = namespace  # Nice option but also unused
        self.wait_for_servers_sec = wait_for_servers_sec

        self.logger_ = self.node_.get_logger()
        self.clock_ = self.node_.get_clock()
        self.client_cb_group_ = (
            MutuallyExclusiveCallbackGroup()
        )  # Change to MECB probablly

        self.planning_pipeline_id_ = ""
        self.planner_id_ = ""

        self.workspace_parameters_ = None

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

        self.max_velocity_scaling_factor_ = 0.1
        self.max_acceleration_scaling_factor_ = 0.1

        self.current_state_ = None
        self.start_state_ = None

        self.initalizing_constraints_ = False

        self.constraints_ = Constraints()

        self.position_constraints_ = []
        self.orientation_constraints_ = []
        self.joint_constraints_ = []
        self.visibility_constraints_ = (
            []
        )  # Add funcitonality for increasd quality submisison -- doesnt exist

        # PUBLISHERS

        self.planning_scene_pub_ = self.node_.create_publisher(
            PlanningScene, self.namespace_ + "planning_scene", 10
        )

        # SUBSCRIBERS

        self.joint_state_sub_ = self.node_.create_subscription(
            JointState,
            self.robot_namespace_ + "joint_states",
            self.jointStateCallback,
            10,
        )

        # ACTION CLIENTS

        self.execute_action_client_ = ActionClient(
            self.node_,
            ExecuteTrajectory,
            self.namespace_ + "execute_trajectory",
            callback_group=self.client_cb_group_,
        )
        if not self.execute_action_client_.wait_for_server(
            timeout_sec=self.wait_for_servers_sec
        ):
            raise RuntimeError(
                "Timeout waiting for execute_trajectory action to \
                    become available"
            )

        self.move_action_client_ = ActionClient(
            self.node_,
            MoveGroup,
            self.namespace_ + "move_action",
            callback_group=self.client_cb_group_,
        )
        if not self.move_action_client_.wait_for_server(
            timeout_sec=self.wait_for_servers_sec
        ):
            raise RuntimeError(
                "Timeout waiting for move_group action to become available"
            )

        # SERVICE CLIENTS

        self.cartesian_path_service_ = self.node_.create_client(
            GetCartesianPath,
            self.namespace_ + "compute_cartesian_path",
            callback_group=self.client_cb_group_,
        )
        if not self.cartesian_path_service_.wait_for_service(
            timeout_sec=self.wait_for_servers_sec
        ):
            raise RuntimeError("Timeout waiting for \
                compute_cartesian_path service")

        self.compute_fk_service_ = self.node_.create_client(
            GetPositionFK,
            self.namespace_ + "compute_fk",
            callback_group=self.client_cb_group_,
        )
        if not self.compute_fk_service_.wait_for_service(
            timeout_sec=self.wait_for_servers_sec
        ):
            raise RuntimeError("Timeout waiting for compute_fk service")

        self.compute_ik_service_ = self.node_.create_client(
            GetPositionIK,
            self.namespace_ + "compute_ik",
            callback_group=self.client_cb_group_,
        )
        if not self.compute_ik_service_.wait_for_service(
            timeout_sec=self.wait_for_servers_sec
        ):
            raise RuntimeError("Timeout waiting for compute_ik service")

        self.get_params_service_ = self.node_.create_client(
            GetPlannerParams,
            self.namespace_ + "get_planner_params",
            callback_group=self.client_cb_group_,
        )
        if not self.get_params_service_.wait_for_service(
            timeout_sec=self.wait_for_servers_sec
        ):
            raise RuntimeError("Timeout waiting for \
                get_planner_params service")

        self.planning_scene_service_ = self.node_.create_client(
            GetPlanningScene,
            self.namespace_ + "get_planning_scene",
            callback_group=self.client_cb_group_,
        )
        if not self.planning_scene_service_.wait_for_service(
            timeout_sec=self.wait_for_servers_sec
        ):
            raise RuntimeError("Timeout waiting for \
                get_planning_scene service")

        self.query_service_ = self.node_.create_client(
            QueryPlannerInterfaces,
            self.namespace_ + "query_planner_interface",
            callback_group=self.client_cb_group_,
        )
        if not self.query_service_.wait_for_service(
            timeout_sec=self.wait_for_servers_sec
        ):
            raise RuntimeError("Timeout waiting for \
                query_planner_interface service")

        self.set_params_service_ = self.node_.create_client(
            SetPlannerParams,
            self.namespace_ + "set_planner_params",
            callback_group=self.client_cb_group_,
        )
        if not self.set_params_service_.wait_for_service(
            timeout_sec=self.wait_for_servers_sec
        ):
            raise RuntimeError("Timeout waiting for \
                set_planner_params service")

    """

    Setters / Getters

    """

    def setAllowedPlanningTime(self, n: float):
        """
        Set the maximum time the motion planner is allowed to plan for.

        Args:
        ----
            n (float): A positive amount of time that the planner is allowed
                to plan for (seconds).

        """
        if isinstance(n, int) or isinstance(n, float):
            if float(n) > 0.0:
                self.allowed_planning_time_ = float(n)
            else:
                self.logger_.error(
                    "Attempt to set allowed_planning_time to impossible time"
                )
        else:
            self.logger_.error(
                "Attempt to set allowed_planning_time to \
                invalid type"
            )

    def getAllowedPlanningTime(self) -> float:
        """
        Get the maximum time the motion planner is allowed to plan for.

        Returns
        -------
            allowed_planning_time_ (float) : A positive amount of time that
            the planner is allowed to plan for (seconds).

        """
        return self.allowed_planning_time_

    def setCanLook(self, n: bool):
        """
        Flag setter to look for more environment info.

        Args:
        ----
            n (bool) : True, if it can look around.

        """
        if isinstance(n, bool):
            self.can_look_ = n
        else:
            self.logger_.error("Attempt to set can_look to invalid type")

    def getCanLook(self) -> bool:
        """
        Flag getter to look for more environment info.

        Returns
        -------
            can_look_ (bool) : True, if it can look around.

        """
        return self.can_look_

    def setCanReplan(self, n: bool):
        """
        Set the flag which allows the planner to replan.

        Args:
        ----
            n (bool) : True, if it can replan

        """
        if isinstance(n, bool):
            self.can_replan_ = n
        else:
            self.logger_.error("Attempt to set can_replan to invalid type")

    def getCanReplan(self):
        """
        Get the flag which allows the planner to replan.

        Returns
        -------
            n (bool) : True, if it can replan

        """
        return self.can_replan_

    def getDefaultPlannerID(self):
        """
        Get the default planning pipeline's id.

        Returns
        -------
            deafult_planner_id (str) : Default planner's id

        """
        # !!! Fix this. Possible in Ros2 but more annoying
        default_planner_id = (
            self.node_.get_parameter("move_group/default_planning_pipeline")
            .get_parameter_value()
            .string_value
        )
        return default_planner_id

    def setGoalJointTolerance(self, n: float):
        """
        Set the tolerance in joint angles for the robot to reach the goal.

        Args:
        ----
            n (float) : Joint angle tolerance (radians).

        """
        if isinstance(n, int) or isinstance(n, float):
            self.goal_joint_tolerance_ = float(n)
        else:
            self.logger_.error(
                "Attempt to set goal_joint_tolerance to \
                invalid type"
            )

    def getGoalJointTolerance(self):
        """
        Get the tolerance in joint angles for the robot to reach the goal.

        Returns
        -------
            goal_joint_tolerance_ (float) : Joint angle tolerance (radians).

        """
        return self.goal_joint_tolerance_

    def setGoalOrientationolerance(self, n: float):
        """
        Set the tolerance in orientation for the robot to reach the goal.

        Args:
        ----
            n (float) : Orientation tolerance.

        """
        if isinstance(n, int) or isinstance(n, float):
            self.goal_orientation_tolerance_ = float(n)
        else:
            self.logger_.error(
                "Attempt to set goal_orientation_tolreance to invalid type"
            )

    def getGoalOrientationolerance(self):
        """
        Get the tolerance in orientation for the robot to reach the goal.

        Returns
        -------
            goal_orientation_tolerance_ (float) : Orientation tolerance.

        """
        return self.goal_orientation_tolerance_

    def setGoalPositionTolerance(self, n: float):
        """
        Set the tolerance in position for the robot to reach the goal.

        Args:
        ----
            n (float) : Position tolerance (m).

        """
        if isinstance(n, int) or isinstance(n, float):
            self.goal_position_tolerance_ = float(n)
        else:
            self.logger_.error(
                "Attempt to set goal_position_tolerance \
                to invalid type"
            )

    def getGoalPositionTolerance(self):
        """
        Get the tolerance in position for the robot to reach the goal.

        Returns
        -------
            goal_position_tolerance_ (float) : Position tolerance (m).

        """
        return self.goal_position_tolerance_

    def setLookAroundAttemps(self, n: int):
        """
        Set the tolerance in position for the robot to reach the goal.

        Args:
        ----
            n (int / float) : Number of times the planner will
                attempt to look around

        """
        if isinstance(n, int):
            if n < 0:
                self.logger_.error(
                    "Attempt to set look attempts negative. Set as positive \
                        instead"
                )
            self.look_around_attempts_ = abs(n)
        else:
            self.logger_.error(
                "Attempt to set look_around_attempts to \
                invalid type"
            )

    def getLookAroundAttemps(self):
        return self.look_around_attempts_

    def setMaxAccelerationScaling(self, n: float):
        """
        Set the maximum acceleration scaling of the motion plan.

        Args:
        ----
            n (float) : Maximum acceleration scaling factor
                ranging from 0.01 to 1.

        """
        if isinstance(n, float) or isinstance(n, int):
            if n >= 0.01:
                if n > 1.0:
                    self.logger_.info(
                        "Maximum acceleration factor exceeding 1.0. Limiting."
                    )
                    self.max_acceleration_scaling_factor_ = 1.0
                else:
                    self.max_acceleration_scaling_factor_ = n
            else:
                self.logger_.info(
                    "Maximum acceleration scaling set below allowed minimum \
                        of 0.01. Disregarding."
                )
        else:
            self.logger_.info(
                "Attempt to set max_acceleration_scaling_factor to invalid \
                    type"
            )

    def getMaxAccelerationScaling(self):
        """
        Get the maximum acceleration scaling of the motion plan.

        Returns
        -------
            max_acceleration_scaling_factor (float) : Maximum acceleration
                scaling factor.

        """
        return self.max_acceleration_scaling_factor_

    def setMaxVelocityScaling(self, n: float):
        """
        Set the maximum velocity scaling of the motion plan.

        Args:
        ----
            n (float / int) : Maximum velocity scaling factor ranging
                from 0.01 to 1.

        """
        if isinstance(n) or isinstance(n, int):
            if n >= 0.01:
                if n > 1.0:
                    self.logger_.info(
                        "Maximum velocity factor exceeding 1.0. Limiting."
                    )
                    self.max_velocity_scaling_factor_ = 1.0
                else:
                    self.max_velocity_scaling_factor_ = n
            else:
                self.logger_.info(
                    "Maximum velocity scaling set below allowed minimum of \
                        0.01. Disregarding."
                )
        else:
            self.logger_.info(
                "Attempt to set max_velocity_scaling_factor to invalid type"
            )

    def getMaxVelocityScaling(self):
        """
        Get the maximum velocity scaling of the motion plan.

        Returns
        -------
            max_velocity_scaling_factor (float) : Maximum velocity
                scaling factor.

        """
        return self.max_velocity_scaling_factor_

    def setNumPlanningAttemps(self, n: int):
        """
        Set the number of times the motion plan is computed.

        Args:
        ----
            n (int) : Number of times the motion plan is computed.
                Should be at least one.

        """
        if isinstance(n, int):
            self.num_planning_attemps_ = n
        else:
            self.logger_.error(
                "Attempt to set num_planning_attempts \
                to invalid type"
            )

    def getNumPlanningAttemps(self) -> int:
        """
        Get the number of times the motion plan is computed.

        Returns
        -------
            num_planning_attempts_ (int) : Number of times the
                motion plan is computed.

        """
        return self.num_planning_attemps_

    def setPlannerId(self, n: str):
        """
        Set the name of the planning algorithm to use.

        If not specified, the default planner of the
            configured planning pipeline is used.

        Args:
        ----
            n (str) : Name of the planning algorithm.

        """
        if isinstance(n, str):
            self.planner_id_ = n
        else:
            self.logger_.error("Attempt to set planner_id to invalid type")

    def getPlannerId(self) -> str:
        """
        Get the name of the planning algorithm to use.

        If not specified, the default planner of
            the configured planning pipeline is used.

        Returns
        -------
            planner_id_(str) : Name of the planning algorithm.

        """
        return self.planner_id_

    def setPlannerParams(self, planner_id: str,
                         group: str, params: PlannerParams):
        """
        Make an asynchronous call to the setPlannerParams service.

        This parameterizes the planner, and sets its config and group.

        Args:
        ----
            planner_id (str) : Planning algorithm to use.
            group (str) : Group of joints on which this planner is operating.
            params (moveit_msgs/PlannerParams) : Parameters as key-value pairs.
            replace (Boolean) : Default false,
                true if the planner should replace its params

        """
        request = SetPlannerParams()
        request.planner_config = planner_id
        request.group = group
        request.params = params

        self.set_params_service_.call_async(request)

    async def getPlannerParams(
        self, planner_id: str, group: str
    ) -> GetPlannerParams.Response:
        """
        Make an asynchronous call to the getPlannerParams service.

        This retrieves the parametrization of the planner corresponding
            to a planner config and planning group.

        Args:
            planner_id (str) : Planning algorithm to use.
            group (str) : Group of joints on which this planner is operating.

        Returns
        -------
            response (moveit_msgs/PlannerParams) :
                Parameters as key-value pairs.

        """
        request = GetPlannerParams()
        request.planner_config = planner_id
        request.group = group

        response = await self.get_params_service_.call_async(request)
        return response

    def setPlanningPipelineID(self, pipeline_id: str):
        """
        Set the name of the planning pipeline to use.

        If not specified, the configured planning pipeline is used.

        Args:
        ----
            pipeline_id (str) : Name of planning pipeline.

        """
        if isinstance(pipeline_id, str):
            if pipeline_id != self.planning_pipeline_id_:
                self.planning_pipeline_id_ = pipeline_id
                self.planner_id_ = ""
        else:
            self.logger_.error(
                "Attempt to set planning_pipeline_id to \
                invalid type"
            )

    def getPlanningPipelineID(self) -> str:
        """
        Get the name of the planning pipeline to use.

        If not specified, the configured planning pipeline is used.

        Returns
        -------
            planning_pipeline_id_ (str) : Name of planning pipeline.

        """
        return self.planning_pipeline_id_

    def setReplanAttempts(self, n: int):
        """
        Set the maximum number of replan attempts.

        In case the plan becomes invalidated during execution.

        Args:
        ----
            n (int) : Maximum number of replan attempts.

        """
        if isinstance(n, int):
            self.replan_attempts_ = n
        else:
            self.logger_.error(
                "Attempt to set replan_attempts to \
                invalid type"
            )

    def getReplanAttempt(self) -> int:
        """
        Get the maximum number of replan attempts.

        In case the plan becomes invalidated during execution.

        Returns
        -------
            replan_attempts_ (int) : Maximum number of replan attempts.

        """
        return self.replan_attempts_

    def setReplanDelay(self, n: float):
        """
        Set the amount of time to wait between replanning attempts.

        Args:
        ----
            n (float) : Time between replanning events (seconds).

        """
        if isinstance(n, int) or isinstance(n, float):
            self.replan_delay_ = float(n)
        else:
            self.logger_.error("Attempt to set replan_delay to invalid type")

    def getReplanDelay(self) -> float:
        """
        Get the amount of time to wait between replanning attempts.

        Returns
        -------
            replan_delay_ (float) : Time between replanning events (seconds).

        """
        return self.replan_delay_

    def setStartState(
        self,
        joint_values: JointState,
        mdof_joint_values: Optional[MultiDOFJointState] = None,
        attached_objects: Optional[AttachedCollisionObject] = None,
        is_diff: Optional[bool] = None,
    ):
        """
        Set the robot's start state in the motion plan.

        Args:
        ----
            joint_values (sensor_msgs/JointState): Starting
                single-dof joint states of the robot.
            mdof_joint_values (sensor_msgs/MultiDOFJointState): Starting
                multi-dof joint states of the robot.
            attached_objects (moveit_msgs/AttachedCollisionObject[]): Attached
                collision objects.
            is_diff (bool): Flag indicating whether this scene is to be
                interpreted as a diff with respect to some other scene.

        """
        self.start_state_ = self.jointsToRobotState(
            joint_values, mdof_joint_values, attached_objects, is_diff
        )

    def setWorkspaceParamaters(
        self,
        minx: float,
        maxx: float,
        miny: float,
        maxy: float,
        minz: float,
        maxz: float,
        frame: Optional[str] = None,
    ):
        """
        Set the cuboidal workspace within which the robot is allowed to move.

        Args:
        ----
            minx (float) : lower x bound of workspace.
            maxx (float) : higher x bound of workspace.
            miny (float) : lower y bound of workspace.
            maxy (float) : higher y bound of workspace.
            minz (float) : lower z bound of workspace.
            maxz (float) : higher z bound of workspace.
            frame (str) : frame in which workspace is defined.
                Defaults to base_link_.

        """
        for value in [minx, miny, minz, maxx, maxy, maxz]:
            if not (isinstance(value, int) or isinstance(value, float)):
                break
        else:
            if frame is None:
                frame = self.base_link_

            self.workspace_parameters_ = WorkspaceParameters()
            self.workspace_parameters_.header.frame_id = frame
            self.workspace_parameters_.header.stamp =\
                self.clock_.now().to_msg()

            self.workspace_parameters_.min_corner.x = minx
            self.workspace_parameters_.min_corner.y = miny
            self.workspace_parameters_.min_corner.z = minz

            self.workspace_parameters_.max_corner.x = maxx
            self.workspace_parameters_.max_corner.y = maxy
            self.workspace_parameters_.max_corner.z = maxz
            return

        # Case of any input is not the right type
        self.logger_().error(
            "Attempt to set workspace parameter coordinate to invalid type"
        )

    """

    Constraint Functions

    """

    def clearAllConstraints(self):
        """Clear all constraints."""
        self.clearPoseConstraints()
        self.clearJointConstraints()

    # Joint Constraints

    def addJointConstraint(
        self, joint_name: str, position: float,
        tol_a=None, tol_b=None, weight=1.0
    ):
        """
        Add a constraint on the position of a joint.

        Args:
        ----
            joint_name (str) : Joint name.
            position (float) : mean position of joint.
            tol_a (float) : Upper bound of joint.
            tol_b (float) : Lower bound of joint.
            weight (float) : Denotes relative importance to other constraints.
                Closer to zero means less important.

        """
        new_jc = JointConstraint()

        if tol_a is None:
            tol_a = self.goal_joint_tolerance_
        if tol_b is None:
            tol_b = self.goal_joint_tolerance_

        new_jc.joint_name = joint_name
        new_jc.position = position
        new_jc.tolerance_above = tol_a
        new_jc.tolerance_below = tol_b
        new_jc.weight = weight

        self.joint_constraints_.append(new_jc)

    def addJointConstraints(
        self, joint_states, tol_a_array=None,
        tol_b_array=None, weight_array=None
    ):
        """
        Add a constraint on the position of multiple joints.

        Args:
        ----
            joint_states (sensor_msgs/JointState) : State of each named
                joint with its corresponding mean position.
            tol_a_array (float[]) : Array of upper bounds of joint_states.
            tol_b_array (float[]) : Array of lower bound of joint_states.
            weight_array (float[]) : Array denoting relative importance
                to other constraints. Closer to zero means less important.
                If set to a scalar, assumes same value for all givens.

        """
        for i in range(len(joint_states.name)):
            if tol_a_array is None:
                tol_a = None
            elif isinstance(tol_a_array, int) or\
                    isinstance(tol_a_array, float):
                tol_a = tol_a_array
            else:
                tol_a = tol_a_array[i]

            if tol_b_array is None:
                tol_b = None
            elif isinstance(tol_b_array, int) or\
                    isinstance(tol_b_array, float):
                tol_b = tol_b_array
            else:
                tol_b = tol_b_array[i]

            if weight_array is None:
                weight = 1.0
            elif isinstance(weight_array, int) or\
                    isinstance(weight_array, float):
                weight = weight_array
            else:
                weight = weight_array[i]

            self.addJointConstraint(
                joint_states.name[i],
                joint_states.position[i],
                tol_a=tol_a,
                tol_b=tol_b,
                weight=weight,
            )

    async def addJointConstraintFromPose(
        self,
        pose_stamped: PoseStamped,
        link: Optional[str] = None,
        start_guess: Optional[RobotState] = None,
        constraints: Optional[Constraints] = None,
        tol_a=None,
        tol_b=None,
        weight=1.0,
    ):
        """
        Add a constraint on the position of multiple joints.

        When given the pose of the end effector when all joints are
            at the mean position.

        Args:
        ----
            pose_stamped (geometry_msgs/PoseStamped) : Time stamped
                pose of the end effector.
            link (str) : End effector link.
            start_guess (moveit_msgs/RobotState) : Initial guess for
                computing joints via IK.
            constraints (moveit_msgs/Constraints) : Set of constraints the
                IK must obey.
            tol_a (float) : The positive tolerance of this constraint
            tol_b (float) : The negative tolerance of this constraint
            weight (float) : The value this constraint has on the path planning

        """
        sol, err = await self.computeIK(pose_stamped, link,
                                        start_guess, constraints)
        if err.val == 1:
            JS = JointState()
            for i in range(len(sol.joint_state.name)):
                for j in self.joint_names_:
                    if sol.joint_state.name[i] == j:
                        JS.name.append(sol.joint_state.name[i])
                        JS.position.append(sol.joint_state.position[i])
            self.addJointConstraints(
                JS, tol_a_array=tol_a, tol_b_array=tol_b, weight_array=weight
            )
        else:
            self.logger_.info(
                "Inverse Kinematics failed with code: {0}".format(err.val)
            )

    def clearJointConstraints(self):
        """Clear joint constraints."""
        self.joint_constraints_ = []

    def mergeJointConstraints(self):
        """
        Merge redundant joint constraints.

        Also discards incompatible joint constraints
        """
        # !!! Fix this: Validate, double check logic
        # For sure doesnt work now if many constraints with same joint_name
        # Possible fix: find all indicies with same joint name
        # Merged back to back with those
        # Separate merge function
        # Seems doable

        merged_jc = []

        for i in range(len(self.joint_constraints_)):
            add = True
            for j in range(i + 1, len(self.joint_constraints_)):
                if (
                    self.joint_constraints_[i].joint_name
                    == self.joint_constraints_[j].joint_name
                ):
                    add = False
                    JC = JointConstraint()
                    first_jc = self.joint_constraints_[i]
                    second_jc = self.joint_constraints_[j]
                    low = max(
                        first_jc.position - first_jc.tolerance_below,
                        second_jc.position - second_jc.tolerance_below,
                    )
                    high = min(
                        first_jc.position + first_jc.tolerance_above,
                        second_jc.position + second_jc.tolerance_above,
                    )
                    if low > high:
                        self.logger_.error(
                            "Incompatiable Joint Constraints for joint "
                            + first_jc.joint_name
                            + ", discarding constraint"
                        )
                    else:
                        JC.joint_name = first_jc.joint_name
                        JC.position = max(
                            low,
                            min(
                                (
                                    first_jc.position * first_jc.weight
                                    + second_jc.position * second_jc.weight
                                )
                                / (first_jc.weight + second_jc.weight)
                            ),
                            high,
                        )
                        JC.weight = (first_jc.weight + second_jc.weight) / 2.0
                        JC.tolerance_above = max(0.0, high - JC.position)
                        JC.tolerance_below = max(0.0, JC.position - low)
                        merged_jc.append(JC)
                    break
            if add:
                merged_jc.append(self.joint_constraints_[i])

        self.joint_constraints_ = merged_jc

    # Orientation Constraints

    def addOrientationConstraint(
        self, pose_stamped: PoseStamped, link=None, tolerance=None, weight=1.0
    ):
        """
        Add an equality constraint on the orientation of a link.

        When given the desired orientation of the link.

        Args:
        ----
            pose_stamped (geometry_msgs/PoseStamped) : Time stamped
                pose of the link.
            link (str) : Constrained link.
            tolerance (float) : XYZ axis tolerance.
            weight (float) : Denotes relative importance to other constraints.
                Closer to zero means less important.

        """
        new_oc = OrientationConstraint()
        new_oc.header = pose_stamped.header
        new_oc.orientation = pose_stamped.pose.orientation
        if link is None:
            new_oc.link_name = self.end_effector_link_
        else:
            new_oc.link_name = link

        if tolerance is None:
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
        """Clear orientation constraints."""
        self.orientation_constraints_ = []

    # Pose Constraints

    def addPoseConstraint(
        self,
        pose_stamped: PoseStamped,
        link=None,
        lin_tol=None,
        ang_tol=None,
        weight=1.0,
    ):
        """
        Add an equality constraint on the pose of a link.

        When given the desired pose of the link.

        Args:
        ----
            pose_stamped (geometry_msgs/PoseStamped) : Time stamped pose
                of the link.
            link (str) : Constrained link.
            lin_tol (float) : Position tolerance (m).
            ang_tol (float) : Orientation tolerance (rad).
            weight (float) : Denotes relative importance to other constraints.
                Closer to zero means less important.

        """
        self.addOrientationConstraint(
            pose_stamped, link=link, tolerance=ang_tol, weight=weight
        )
        self.addPositionConstraint(
            pose_stamped, link=link, tolerance=lin_tol, weight=weight
        )

    def addPoseConstraints(
        self,
        pose_stamped_array: list[PoseStamped],
        link_array: list[str],
        lin_tol_array=None,
        ang_tol_array=None,
        weight_array=1.0,
    ):
        """
        Add an equality constraint on the pose of multiple links.

        When given the desired pose of these links.

        Args:
        ----
            pose_stamped_array (geometry_msgs/PoseStamped[]) : Array of time
                stamped poses of links.
            link_array (str[]) : Array of constrained links.
            lin_tol_array (float[]) : Array of position tolerances (m).
            ang_tol_array (float[]) : Array of orientation tolerances (rad).
            weight_array (float[]) : Array denoting relative
                importance to other constraints. Closer to zero
                means less important. If set to a scalar,
                assumes same value for all givens.

        """
        for i in range(len(pose_stamped_array)):
            if lin_tol_array is None:
                lin_tol = None
            elif isinstance(lin_tol_array, int) or\
                    isinstance(lin_tol_array, float):
                lin_tol = lin_tol_array
            else:
                lin_tol = lin_tol_array[i]

            if ang_tol_array is None:
                ang_tol = None
            elif isinstance(ang_tol_array, int) or\
                    isinstance(ang_tol_array, float):
                ang_tol = ang_tol_array
            else:
                ang_tol = ang_tol_array[i]

            if weight_array is None:
                weight = 1.0
            elif isinstance(weight_array, int) or\
                    isinstance(weight_array, float):
                weight = weight_array
            else:
                weight = weight_array[i]
            self.addPoseConstraint(
                pose_stamped_array[i],
                link=link_array[i],
                lin_tol=lin_tol,
                ang_tol=ang_tol,
                weight=weight,
            )

    async def addPoseConstraintFromJoints(
        self,
        joint_state: list[JointState],
        base_frame=None,
        lin_tol_array=None,
        ang_tol_array=None,
        weight_array=1.0,
        link_names=None,
        mdof_joint_values=None,
        attached_objects=None,
        is_diff=None,
    ):
        """
        Add an equality constraint on the pose of multiple links.

        When given the joint states.

        Args:
        ----
            joint_state (sensor_msgs/JointState) : State of each named
                single-dof joint of the robot.
            base_frame (float[]) : Base frame of the robot
            lin_tol_array (float[]) : Array of position tolerances (m).
            ang_tol_array (float[]) : Array of orientation tolerances (rad).
            weight_array (float[]) : Array denoting relative importance
                to other constraints. Closer to zero means less important.
            link_names (str[]) : Array of constrained links.
            mdof_joint_values (sensor_msgs/MultiDOFJointState) : State of each
                named multi-dof joint of the robot.
            attached_objects (moveit_msgs/AttachedCollisionObject[]): Attached
                collision objects.
            is_diff (bool) : Flag indicating whether this scene is to be
            interpreted as a diff with respect to some other scene.

        """
        poses, names, err = await self.computeFK(
            joint_values=joint_state,
            base_frame=base_frame,
            link_names=link_names,
            mdof_joint_values=mdof_joint_values,
            attached_objects=attached_objects,
            is_diff=is_diff,
        )
        if err.val == 1:
            self.addPoseConstraints(
                poses,
                names,
                lin_tol_array=lin_tol_array,
                ang_tol_array=ang_tol_array,
                weight_array=weight_array,
            )
        else:
            self.logger_.info(
                "Forward Kinematics failed with code: {0}".format(err.val)
            )

    def clearPoseConstraints(self):
        """Clear pose constraints."""
        self.clearOrientationConstraints()
        self.clearPositionConstraints()

    # Position Constraints

    def addPositionConstraint(
        self,
        pose_stamped: PoseStamped,
        link=None,
        offset=None,
        tolerance=None,
        weight=1.0,
    ):
        """
        Add an equality constraint on the position of a link.

        When given the desired position of the link.

        Args:
        ----
            pose_stamped (geometry_msgs/PoseStamped) : Time stamped pose
                of the link.
            link (str) : Constrained link.
            offset (float) : Offset that should be added to the target
            tolerance (float) : Position tolerance (m).
            weight (float) : Denotes relative importance to other constraints.
                Closer to zero means less important.

        """
        new_pc = PositionConstraint()
        new_pc.header = pose_stamped.header

        if not (offset is None):
            new_pc.target_point_offset.x = offset.x
            new_pc.target_point_offset.y = offset.y
            new_pc.target_point_offset.z = offset.z

        if link is None:
            new_pc.link_name = self.end_effector_link_
        else:
            new_pc.link_name = link

        if tolerance is None:
            tolerance = self.goal_position_tolerance_

        BV = BoundingVolume()

        SP = SolidPrimitive()
        SP.type = 2
        SP.dimensions = [tolerance]

        BV.primitives = [SP]
        BV.primitive_poses = [pose_stamped.pose]

        new_pc.constraint_region = BV
        new_pc.weight = weight

        self.position_constraints_.append(new_pc)

    def clearPositionConstraints(self):
        """Clear position constraints."""
        self.position_constraints_ = []

    """

    Action functions

    """

    async def addCollisionObject(
        self, shape: SolidPrimitive, pose_stamped: PoseStamped
    ):
        """
        Add a rigid object to the planning scene.

        Args:
        ----
            shape (shape_msgs/SolidPrimitive[]) : Shapes that
                the body consists of.
            pose_stamped (geometry_msgs/Pose[]) : Respective poses
                of these shapes.

        """
        scene = await self.getPlanningScene()

        obj = CollisionObject()
        obj.header = pose_stamped.header
        obj.pose = pose_stamped.pose
        obj.primitives = [shape]
        obj.primitive_poses = [Pose()]

        scene.world.collision_objects.append(obj)

        self.planning_scene_pub_.publish(scene)

    def cleanUp(self):
        """Clean up after planning."""
        self.clearAllConstraints()
        self.start_state_ = None

    async def computeFK(
        self,
        joint_values,
        base_frame=None,
        link_names=None,
        mdof_joint_values=None,
        attached_objects=None,
        is_diff=None,
    ) -> tuple[PoseStamped, list[str], MoveItErrorCodes]:
        """
        Compute forward kinematics.

        Args:
            joint_values (sensor_msgs/JointState) : State of each
                named joint of the robot.
            base_frame (str) : Name of base_frame. Defaults to base_link_
            link_names (str[]) : Vector of link names for
                which forward kinematics must be computed.
            mdof_joint_values (sensor_msgs/MultiDOFJointState) : State of each
                multi-dof joint of the robot.
            attached_objects (moveit_msgs/AttachedCollisionObject[]): Attached
                collision objects.
            is_diff (bool) : Flag indicating whether this scene
                is to be interpreted as a
                diff with respect to some other scene.

        Returns
        -------
            poses (geometry_msgs/PoseStamped[]) : The resultant vector of
                PoseStamped messages that contains the
                (stamped) poses of the requested links.
            names (str[]) : The list of link names corresponding to the poses
            err (moveit_msgs/MoveItErrorCodes) : MoveIt! error codes.

        """
        req = GetPositionFK.Request()
        if base_frame is None:
            base_frame = self.base_link_
        req.header.frame_id = base_frame

        if link_names is None:
            link_names = [self.end_effector_link_]
        req.fk_link_names = link_names

        RS = self.jointsToRobotState(
            joint_values, mdof_joint_values, attached_objects, is_diff
        )
        req.robot_state = RS

        response: GetPositionFK.Response = await\
            self.compute_fk_service_.call_async(
                req
            )
        poses = response.pose_stamped
        names = response.fk_link_names
        err = response.error_code
        return poses, names, err

    async def computeIK(
        self, pose_stamped, link=None, start_guess=None, constraints=None
    ) -> tuple[RobotState, MoveItErrorCodes]:
        """
        Compute inverse kinematics.

        Args:
            pose_stamped (geometry_msgs/PoseStamped) : Time stamped pose of
                the end effector.
            link (str) : Given link.
            start_guess (moveit_msgs/RobotState) : Initial guess for
                computing joints via IK.
            constraints (moveit_msgs/Constraints) : Set of constraints
                the IK must obey.

        Returns
        -------
            sol (moveit_msgs/RobotState) : Robot state solution in the
                same order as start_guess.
            err (moveit_msgs/MoveItErrorCodes) : MoveIt! error codes.

        """
        # Does not support multiple link submissions
        req = GetPositionIK.Request().ik_request
        req.group_name = self.group_name_
        if start_guess is None:
            start_guess = self.jointsToRobotState(self.current_state_)
        req.robot_state = start_guess
        if not (constraints is None):
            req.constraints = constraints
        if link is None:
            link = self.end_effector_link_
        req.ik_link_name = link
        req.pose_stamped = pose_stamped
        response = await self.compute_ik_service_.call_async(
            GetPositionIK.Request(ik_request=req)
        )
        sol = response.solution
        err = response.error_code
        return sol, err

    def constructMotionPlanRequest(self) -> MotionPlanRequest:
        """
        Construct a motion plan request from the path planner's members.

        Returns
        -------
            request (moveit_msgs/MotionPlanRequest) : Motion plan request from
                the path planner's members.

        """
        request = MotionPlanRequest()
        request.group_name = self.group_name_
        request.num_planning_attempts = self.num_planning_attemps_
        request.max_velocity_scaling_factor = self.max_velocity_scaling_factor_
        request.max_acceleration_scaling_factor =\
            self.max_acceleration_scaling_factor_
        request.allowed_planning_time = self.allowed_planning_time_
        request.pipeline_id = self.planning_pipeline_id_
        request.planner_id = self.planner_id_
        request.workspace_parameters = self.workspace_parameters_
        request.start_state = self.start_state_
        self.constraints_.position_constraints = self.position_constraints_
        self.constraints_.orientation_constraints =\
            self.orientation_constraints_
        self.constraints_.joint_constraints = self.joint_constraints_

        # !!! Fix this function
        # self.constraints_ = self.mergeJointConstraints()
        request.goal_constraints = [self.constraints_]
        return request

    def constructPlannerOptions(self, plan_only=True) -> PlanningOptions:
        """
        Construct a planning options message.

        Args:
            plan_only (bool) : If true, the action returns an executable plan
                in the response but does not attempt an execution.

        Returns
        -------
            options (moveit_msgs/PlanningOptions) : Planning options message.

        """
        options = PlanningOptions()
        options.plan_only = plan_only
        options.look_around = self.can_look_
        options.look_around_attempts = self.look_around_attempts_
        options.replan = self.can_replan_
        options.replan_attempts = self.replan_attempts_
        options.replan_delay = self.replan_delay_
        return options

    async def executeTrajectory(self, trajectory):
        """
        Execute a given joint-space robot trajectory.

        Does this by calling the ExecuteTrajectory action through
            the execute_trajectory client.

        Args:
            trajectory (moveit_msgs/RobotTrajectory) : If true, the action
                returns an executable plan in the response
                but does not attempt an execution.

        Returns
        -------
            result (ExecuteTrajectory.Result) : result of the execute
                trajectory action.
            status (GoalStatus) : Indicates goal status.

        """
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
            self.logger_.info("Goal failed with status: {0}".format(status))
        return result, status

    async def getPlanningScene(self, components=PlanningSceneComponents()):
        """
        Get parts of the planning scene.

        Does this by calling the GetPlanningScene service through
            the get_planning_scene client.

        All scene components are returned if none are specified.

        Args:
            components (moveit_msgs/PlanningSceneComponents) : Components that
                make up the planning scene. Bitfield that
                specifies the parts of the planning scene that are of interest.

        Returns
        -------
            scene (moveit_msgs/PlanningScene) : Planning scene.

        """
        planning_scene = await self.planning_scene_service_.call_async(
            GetPlanningScene.Request(components=components)
        )
        return planning_scene.scene

    def jointStateCallback(self, msg):
        """
        Is the callback function for subscriber to topic joint_states.

        Updates current state of joints.
        Whitelisted to only allow joints passed in the RobotModel.

        Args:
        ----
            msg (sensor_msgs/JointState) : Timestamped state of joints.

        """
        self.current_state_ = JointState()
        self.current_state_.header = msg.header

        for i in range(len(msg.name)):
            for j in self.joint_names_:
                if msg.name[i] == j:
                    self.current_state_.name.append(msg.name[i])
                    self.current_state_.position.append(msg.position[i])
                    self.current_state_.velocity.append(msg.velocity[i])
                    self.current_state_.effort.append(msg.effort[i])

    def jointsToRobotState(
        self,
        joint_values: JointState,
        mdof_joint_values: Optional[MultiDOFJointState] = None,
        attached_objects: Optional[AttachedCollisionObject] = None,
        is_diff: Optional[bool] = None,
    ):
        """
        Convert a JointState message to a RobotState message.

        Args:
        ----
            joint_values (sensor_msgs/JointState) : Single-dof joint
                states of the robot.
            mdof_joint_values (sensor_msgs/MultiDOFJointState) : Multi-dof
                joint states of the robot.
            attached_objects (moveit_msgs/AttachedCollisionObject[]): Attached
                collision objects.
            is_diff (bool) : Flag indicating whether this scene is to
                be interpreted as a diff with respect to some other scene.

        """
        RS = RobotState()
        RS.joint_state = joint_values
        if not (mdof_joint_values is None):
            RS.multi_dof_joint_state = mdof_joint_values
        if not (attached_objects is None):
            RS.attached_collision_objects = attached_objects
        if not (is_diff is None):
            RS.is_diff = is_diff
        return RS

    async def makeMotionPlanRequest(
        self, plan_only=True, start_state=None,
        eef_pose_stamped=None, clean=True
    ) -> tuple[MoveGroup.Result, GoalStatus]:
        """
        Make a motion plan request to the MoveGroup action.

        Args:
            plan_only (bool) : If true, the action returns an
                executable plan in the
                response but does not attempt an execution.
            start_state (moveit_msgs/RobotState) : A given start state,
                ideally given when only planning and not executing.
            eef_pose_stamped (geometry_msgs/PoseStamped) : TimeStamped
                position of the end effector.
            clean (bool) : Flag specifiying whether to clean up
                (clear all constraints and set start to None) or
                not after planning.

        Returns
        -------
            result (MoveGroup.Result) : Result of the MoveGroup action.
            status (GoalStatus) : Indicates goal status.

        """
        if not (eef_pose_stamped is None):
            self.addPositionConstraint(eef_pose_stamped)
            self.addOrientationConstraint(eef_pose_stamped)

        # If plan_only = False (executing trajectory right after),
        # MUST start at current state
        # If given a start_state, will override the current start_state_.
        if (start_state is None and self.start_state_ is None) or\
                not (plan_only):
            self.setStartState(self.current_state_)
        elif not (start_state is None):
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
            self.logger_.info("Goal failed with status: {0}".format(status))
        if clean:
            self.cleanUp()
        return result, status
