#!/usr/bin/env python3

# core python libraries
import time
import copy
import math
import code

# ros imports
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

# moveit imports
from moveit.core.robot_state import RobotState
from moveit.core.planning_interface import MotionPlanResponse
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
    )

# ros2 messages
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    CollisionObject, AttachedCollisionObject, PlanningScene,
    Constraints, OrientationConstraint, PositionConstraint,
    )
from sensor_msgs.msg import JointState

from shape_msgs.msg import SolidPrimitive

# our custom messages
from demo_planning_msgs.srv import PlanningService, PlanningService_Request, PlanningService_Response

# xarm messages
from xarm_msgs.srv import VacuumGripperCtrl


def euler_to_quaternion(yaw, pitch, roll):
    """
    Convert Euler angles (yaw, pitch, roll) to a quaternion.
    Assumes ZYX intrinsic rotation order.
    Angles should be in radians.
    """
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return [qw, qx, qy, qz]


def log_positions(robot_state, logger):
    positions = robot_state.get_joint_group_positions("lite6")
    logger.info(f"positions: {positions}")


class RobotLogic():

    def __init__(self, moveit, logger, pose_link):
        self.moveit = moveit
        self.logger = logger
        self.pose_link = pose_link
        self.no_constraints = Constraints()
        self.multi_params = MultiPipelinePlanRequestParameters(
                #the names of the planners reflect those in
                #motion_planning_python_api_tutorial.yaml
                moveit, ["ompl_rrtc", "pilz_ptp", "pilz_lin", "ompl_rrt_star", "stomp_planner"])


    def attach_physical_object(self, obj):
        pass

    def detach_physical_object(self):
        pass

    def move_to_object(self, obj):
        pass

    def move_to_drop_location(self, location):
        pass

    def plan_and_execute(
            self,
            single_plan_parameters=None,
            multi_plan_parameters=None,
            sleep_time=0.0,
            ):
        """Helper function to plan and execute a motion."""
        # plan to goal
        if multi_plan_parameters is not None:
            plan_result = self.arm.plan(
                multi_plan_parameters=multi_plan_parameters
                )
        elif single_plan_parameters is not None:
            plan_result = self.arm.plan(
                single_plan_parameters=single_plan_parameters
                )
        else:
            plan_result = self.arm.plan()

        # execute the plan
        if plan_result:
            self.logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
            self.logger.info("Execute finished!")
        else:
            self.logger.error("Planning failed")

        time.sleep(sleep_time)

    # plan to goal, with multiple planners
    def plan(self,):
        return self.arm.plan(multi_plan_parameters=self.multi_params)

    def plan_to(self, start: JointState, goal: PoseStamped) -> MotionPlanResponse:
        robot_model = self.moveit.get_robot_model()
        start_state = RobotState(robot_model)
        start_state.joint_positions = dict(zip(start.name, start.position))
        self.arm.set_start_state(robot_state = start_state)
        self.arm.set_goal_state(pose_stamped_msg = goal, pose_link = self.pose_link)
        self.logger.info(f"PLAN TO: args {repr(start)} {repr(goal)}")
        plan = self.plan()
        return plan

    def move_to(self, obj_pose: Pose, frame: str = "world", constraints: Constraints = None):
        self.logger.info(f"object coordinates: {
                               obj_pose.position.x,
                               obj_pose.position.y,
                               obj_pose.position.z}")

        self.arm.set_start_state_to_current_state()
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = frame

        q = euler_to_quaternion(math.radians(0), math.radians(180), math.radians(0))

        pose_goal.pose.orientation.w = q[0]
        pose_goal.pose.orientation.x = q[1]
        pose_goal.pose.orientation.y = q[2]
        pose_goal.pose.orientation.z = q[3]

        pose_goal.pose.position.x = obj_pose.position.x
        pose_goal.pose.position.y = obj_pose.position.y
        pose_goal.pose.position.z = obj_pose.position.z

        if constraints is None:
            constraints = self.no_constraints

        self.arm.set_path_constraints(constraints)
        self.arm.set_goal_state(
                pose_stamped_msg=pose_goal,
                pose_link=self.pose_link,)
        self.plan_and_execute(multi_plan_parameters=self.multi_params)


class RobotWithoutGripper(RobotLogic):

    def __init__(self, moveit, logger, pose_link):
         super().__init__(moveit, logger, pose_link)
         self.arm = self.moveit.get_planning_component("lite6")

    def move_to_object(self, obj_pose: Pose):
         self.move_to(obj_pose)

    def move_to_drop_location(self, location: Pose):
        self.move_to(location, "link_base")


class RobotWithVacuumGripper(RobotLogic):

    def __init__(self, moveit, logger, pose_link, srv_cli):
        super().__init__(moveit, logger, pose_link)
        self.cli = srv_cli
        self.arm = self.moveit.get_planning_component("lite6")
        self.no_constraints = Constraints()
        self.orientation_constraint = RobotLogic.get_orientation_constraint(pose_link)



class DemoPlanningService():

    def __init__(self):
        rclpy.init()
        self.logger = get_logger("planning_service")

        # instantiate MoveItPy instance and get planning component
        self.moveit = MoveItPy(node_name="planning_service")
        self.logger.info("Planning Service MoveItPy instance created")


        self.node = rclpy.create_node("planning_service_node")

        ## Setup necessary parameters
        ## vacuum gripper
        self.node.declare_parameter("add_vacuum_gripper", False)
        self.vacuum_gripper = self.node.get_parameter("add_vacuum_gripper").get_parameter_value().bool_value

        ## robot_ip
        self.node.declare_parameter("robot_ip", "fake")
        self.robot_ip = self.node.get_parameter("robot_ip").get_parameter_value().string_value

        self.logger.info(f"CONFIGURATION: vacuum_gripper {self.vacuum_gripper}, robot_ip {self.robot_ip}")

        # setup end effector link
        if self.vacuum_gripper:
            self.pose_link = "link_tcp"
            self.allowed_to_touch = "uflite_vacuum_gripper_link"
        else:
            self.pose_link = "link_eef"
            self.allowed_to_touch = "link6"

        # finally initialize appropriate robot logic
        if self.vacuum_gripper and self.robot_ip != "fake":
            self.logger.info(f"this is real arm, with vacuum gripper")
            srv_cli = self.node.create_client(VacuumGripperCtrl, "/ufactory/set_vacuum_gripper")

            while not srv_cli.wait_for_service(timeout_sec=1.0):
                self.logger.info(f"waiting for xarm VacuumGripperCtrl to come up")

            self.robot_logic = RobotWithVacuumGripper(self.moveit, self.logger, self.pose_link, srv_cli)

        elif self.vacuum_gripper and self.robot_ip == "fake":
            self.logger.info(f"we have a fake robot, but with a vacuum gripper")
            self.robot_logic = RobotWithoutGripper(self.moveit, self.logger, self.pose_link)

        else:
            #we have a physical or fake robot without vacuum gripper
            self.robot_logic = RobotWithoutGripper(self.moveit, self.logger, self.pose_link)

        self.srv = self.node.create_service(PlanningService, 'plan_to_goal', self.plan_to_goal)

    #the callback for the planning service
    def plan_to_goal(self, request:PlanningService_Request, response:PlanningService_Response) -> PlanningService_Response:
        try:
            plan = self.robot_logic.plan_to(request.start, request.goal)
            self.logger.info(f"plan details: {type(plan)}, {plan.error_code.val}, {str(plan.trajectory)}")
            response.trajectories = [plan.trajectory.get_robot_trajectory_msg()]
        except Exception as ex:
            self.logger.warn(str(ex))
            response.trajectories = []
        return response


def main():
    planning_service = DemoPlanningService()
    planning_service.logger.info("demo planning service, start spinning!")
    rclpy.spin(planning_service.node)


if __name__ == "__main__":
    main()
