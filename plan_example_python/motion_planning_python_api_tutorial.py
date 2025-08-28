#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time
import code

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
    )


def plan_and_execute(
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
        ):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
            )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
            )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
        logger.info("Execute finished!")
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)

def log_positions(robot_state, logger):
    positions = robot_state.get_joint_group_positions("lite6")
    logger.info(f"positions: {positions}")

def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    # marios
    # just to make sure everything is set up before we start doing things
    time.sleep(5)
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    cobot = MoveItPy(node_name="moveit_py")
    arm = cobot.get_planning_component("lite6")
    logger.info("MoveItPy instance created")
    time.sleep(5)
    logger.info("Sleeping for 5 secs...")

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################

    # set plan start state using predefined state
    arm.set_start_state_to_current_state()

    # set pose goal using predefined state
    arm.set_goal_state(configuration_name="home")

    # plan to goal
    plan_and_execute(cobot, arm, logger, sleep_time=1.0)

    ############################################################################
    ## Plan 2 - set goal state with RobotState object
    ############################################################################

    ## instantiate a RobotState instance using the current robot model
    robot_model = cobot.get_robot_model()
    robot_state = RobotState(robot_model)
    log_positions(robot_state, logger)


    # randomize the robot state
    robot_state.set_to_random_positions()

    # set plan start state to current state
    arm.set_start_state_to_current_state()

    # set goal state to the initialized robot state
    logger.info("Set goal state to the initialized robot state")
    arm.set_goal_state(robot_state=robot_state)

    # plan to goal
    plan_and_execute(cobot, arm, logger)

    log_positions(robot_state, logger)

    ###########################################################################
    # Plan 3 - set goal state with PoseStamped message
    ###########################################################################
    # TODO figure out the coordinate system

    # set plan start state to current state
    arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    from geometry_msgs.msg import PoseStamped

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "link_base"
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.orientation.x = 1.0
    #pose_goal.pose.orientation.y = .5
    #pose_goal.pose.orientation.z = -1.0
    #these are METERS! don't push your luck
    pose_goal.pose.position.x = 0.2
    pose_goal.pose.position.y = 0.0
    pose_goal.pose.position.z = 0.4
    arm.set_goal_state(
        # that or joint6_flange maybe?
        pose_stamped_msg=pose_goal, pose_link="link6")

    # plan to goal
    plan_and_execute(cobot, arm, logger, sleep_time=8.0)

    ############################################################################
    ## Plan 4 - set goal state with constraints
    ############################################################################

    ## set plan start state to current state
    #arm.set_start_state_to_current_state()

    ## set constraints message
    #from moveit.core.kinematic_constraints import construct_joint_constraint

    #joint_values = {
    #    "joint1": -55.4/180.,
    #    "joint2": 27.0/180.,
    #    "joint3": 45.0/180.,
    #    "joint4": 12.0/180.,
    #    "joint5": 17.0/180.,
    #    "joint6": -12.8/180,
    #    }

    #robot_model = cobot.get_robot_model()
    #robot_state = RobotState(robot_model)

    #robot_state.joint_positions = joint_values
    #joint_constraint = construct_joint_constraint(
    #    robot_state=robot_state,
    #    joint_model_group=cobot.get_robot_model().get_joint_model_group("lite6"),
    #    )
    #arm.set_goal_state(motion_plan_constraints=[joint_constraint])

    ## plan to goal
    #plan_and_execute(cobot, arm, logger, sleep_time=8.0)

    #log_positions(robot_state, logger)

    ############################################################################
    ## Plan 5 - Planning with Multiple Pipelines simultaneously
    ############################################################################

    ## set plan start state to current state
    #arm.set_start_state_to_current_state()

    ## set pose goal with PoseStamped message
    #arm.set_goal_state(configuration_name="lookup")

    ## initialise multi-pipeline plan request parameters
    #multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
    #    #cobot, ["ompl_rrtc", "pilz_lin", "chomp_planner"]
    #    cobot, ["chomp_planner"]
    #    )

    ## plan to goal
    #plan_and_execute(
    #    cobot,
    #    arm,
    #    logger,
    #    multi_plan_parameters=multi_pipeline_plan_request_params,
    #    sleep_time=8.0,
    #    )

    #code.interact(local=locals())

    print("WE'RE DONE!")
    time.sleep(100)


if __name__ == "__main__":
    main()
