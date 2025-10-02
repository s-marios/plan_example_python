#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

# core python libraries
import time
import math
import code

# generic ros libraries
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

# moveit related library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
    )

# ros2 messages
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive


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
    # just to make sure everything is set up before we start doing things
    time.sleep(5)
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    #rclpy.create_node(...)

    # instantiate MoveItPy instance and get planning component
    moveit = MoveItPy(node_name="moveit_py")
    arm = moveit.get_planning_component("lite6")
    logger.info("MoveItPy instance created")

    ### Setup a second node and a publisher
    node = rclpy.create_node("helper_node")
    logger.info("created helper node!")


    ###########################################################################
    # Plan 1 - Move to a predefined state
    ###########################################################################

    # set plan start state using predefined state
    arm.set_start_state_to_current_state()

    # set pose goal using predefined state
    arm.set_goal_state(configuration_name="home")

    # plan to goal
    plan_and_execute(moveit, arm, logger, sleep_time=1.0)


    ############################################################################
    ## Plan 2 - Move to a random state
    ############################################################################

    ## instantiate a RobotState instance using the current robot model
    robot_model = moveit.get_robot_model()
    robot_state = RobotState(robot_model)
    log_positions(robot_state, logger)


    # randomize the robot state
    robot_state.set_to_random_positions()

    # set plan start state to current state
    arm.set_start_state_to_current_state()

    # set goal state to the initialized robot state
    logger.info("Set goal state to the initialized robot state")
    arm.set_goal_state(robot_state=robot_state)

    # comment out the next line if you want to skip it
    plan_and_execute(moveit, arm, logger)

    log_positions(robot_state, logger)


    ###########################################################################
    # Plan 3 - Set goal state using constraints
    ###########################################################################

    # set plan start state to current state
    arm.set_start_state_to_current_state()

    # set constraints message
    from moveit.core.kinematic_constraints import construct_joint_constraint

    joint_values = {
        "joint1": math.radians(-55.4),
        "joint2": math.radians(27.0),
        "joint3": math.radians(45.0),
        "joint4": math.radians(12.0),
        "joint5": math.radians(17.0),
        "joint6": math.radians(-12.8),
        }

    robot_model = moveit.get_robot_model()
    robot_state = RobotState(robot_model)

    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=moveit.get_robot_model().get_joint_model_group("lite6"),
        )
    arm.set_goal_state(motion_plan_constraints=[joint_constraint])

    # plan to goal
    plan_and_execute(moveit, arm, logger)

    log_positions(robot_state, logger)

    ###########################################################################
    # Plan 4 - Move to specific coordinate
    # points on a cube, effector orientaion pointing "outwards" from the
    # (roughly) center of the cube
    ###########################################################################

    # set pose goal with PoseStamped message
    from geometry_msgs.msg import PoseStamped

    angles = [
        [225, 135, 0],
        [315, 135, 0],
        [45, 135, 0],
        [135, 135, 0],
        [135, 45, 0],
        [45, 45, 0],
        [315, 45, 0],
        [225, 45, 0],
            ]

    quaternions = [euler_to_quaternion(
        math.radians(angle[0]),
        math.radians(angle[1]),
        math.radians(angle[2])
        ) for angle in angles]

    cube = [
        [-0.25, -0.25, 0.25],
        [0.25, -0.25, 0.25],
        [0.25, 0.25, 0.25],
        [-0.25, 0.25, 0.25],
        [-0.25, 0.25, 0.6],
        [0.25, 0.25, 0.6],
        [0.25, -0.25, 0.6],
        [-0.25, -0.25, 0.6],
            ]

    # Move through all waypoints
    for i, (p, q) in enumerate(zip(cube, quaternions), start=1):
        arm.set_start_state_to_current_state()
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "link_base"
        pose_goal.pose.orientation.w = q[0]
        pose_goal.pose.orientation.x = q[1]
        pose_goal.pose.orientation.y = q[2]
        pose_goal.pose.orientation.z = q[3]

        pose_goal.pose.position.x = p[0]
        pose_goal.pose.position.y = p[1]
        pose_goal.pose.position.z = p[2]

        logger.info(f"Moving to point {i}: x={p[0]}, y={p[1]}, z={p[2]}")
        arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")
        plan_and_execute(moveit, arm, logger, sleep_time=0.5)


    ###########################################################################
    # Plan 5 - Spawn virtual objects Using MoveIt!
    ###########################################################################

    planning_scene_monitor = moveit.get_planning_scene_monitor()
    logger.info(f"type of planning scene is: {type(planning_scene_monitor)}")

    object_positions = [
        (-0.15, 0.25, 0.0),
        (-0.15, -0.25, 0.0),
    ]
    object_dimensions = [
        (0.05, 0.05, 0.05),
        (0.05, 0.05, 0.05),
    ]

    # a reference to the green boxes so we can delete them later
    green_boxes = None

    planning_scene_monitor = moveit.get_planning_scene_monitor()
    with planning_scene_monitor.read_write() as scene:

        collision_object = CollisionObject()
        green_boxes = collision_object

        collision_object.header.frame_id = "link_base"
        collision_object.id = "boxes"

        for position, dimensions in zip(object_positions, object_dimensions):
            box_pose = Pose()
            box_pose.position.x = position[0]
            box_pose.position.y = position[1]
            box_pose.position.z = position[2]

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = dimensions
            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(box_pose)
            # the boxes appear even if the following line is commented out
            collision_object.operation = CollisionObject.ADD

        scene.apply_collision_object(collision_object)
        scene.current_state.update()

        logger.info("boxes added to the scene!")
    time.sleep(10)

    ############################################################################
    ## Plan 6 - Spawn virtual objects using raw messages
    ############################################################################


    pspub = node.create_publisher(PlanningScene, "/planning_scene", 10)

    collision_object = CollisionObject()
    collision_object.header.frame_id = "link6" #relative to the eef
    collision_object.id = "lightsaber"

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [0.03, 0.03, 0.16]
    collision_object.primitives.append(primitive)

    object_pose = Pose()
    object_pose.position.x = 0.0
    object_pose.position.y = 0.0
    object_pose.position.z = 0.08
    collision_object.primitive_poses.append(object_pose)

    attached_object = AttachedCollisionObject()
    attached_object.object = collision_object
    attached_object.link_name = "link6"
    attached_object.object.operation = CollisionObject.ADD
    attached_object.touch_links = ["link6"]

    planning_scene = PlanningScene()
    planning_scene.is_diff = True
    planning_scene.robot_state.attached_collision_objects.append(attached_object)
    planning_scene.robot_state.is_diff = True

    pspub.publish(planning_scene)
    rclpy.spin_once(node, timeout_sec=1.0)

    time.sleep(10)

    ##OR EQUIVALENTLY WITH MOVEIT
    #planning_scene_monitor = moveit.get_planning_scene_monitor()
    #with planning_scene_monitor.read_write() as scene:
    #    scene.process_attached_collision_object(attached_object)
    #    scene.current_state.update()


    ############################################################################
    ## Plan 7 - Remove attached virtual object
    ############################################################################

    # grab a copy of the current scene to recall it later
    prev_scene = None
    with planning_scene_monitor.read_only() as scene:
        prev_scene = scene.planning_scene_message

    # remove the lightsaber
    planning_scene = PlanningScene()
    planning_scene.is_diff = True
    attached_object.object.operation = CollisionObject.REMOVE
    ## detatch object
    planning_scene.robot_state.attached_collision_objects.append(attached_object)
    planning_scene.robot_state.is_diff = True
    ## remove it from the world completely
    # you can comment out the line below and see that the object is still in the world
    planning_scene.world.collision_objects.append(attached_object.object)
    pspub.publish(planning_scene)
    rclpy.spin_once(node, timeout_sec=1.0)

    time.sleep(10)

    # houdini, make the lightsaber appear again!
    pspub.publish(prev_scene)
    rclpy.spin_once(node, timeout_sec=1.0)

    time.sleep(3)

    # remove the green boxes
    planning_scene_monitor = moveit.get_planning_scene_monitor()
    with planning_scene_monitor.read_write() as scene:
        green_boxes.operation = CollisionObject.REMOVE
        scene.apply_collision_object(green_boxes)
        scene.current_state.update()

        ## this did not the way I expected it to
        ## not exactly sure that the attached object gets removed
        #scene.process_attached_collision_object(attached_object)
        #scene.current_state.update()

    ## this didn't seem to work either..
    #    robot_state.clear_attached_bodies()
    #    robot_state.update()



#    ###########################################################################
#    # Plan X - set goal state with PoseStamped message
#    # Use this when you want to induce positioning errors, testing accuracy
#    ###########################################################################
#    # TODO figure out the coordinate system
#
#    # set plan start state to current state
#
#    # set pose goal with PoseStamped message
#    from geometry_msgs.msg import PoseStamped
#
#    ypra = [ ]
#    ypra.extend([ [0, 0, i*45] for i in range(1,8)])
#    #these tend to produce out-of-bounds, leave them out for the time being
#    ypra.extend([ [i*45, 0, 0] for i in range(1,8)])
#    ypra.extend([ [0, i*45, 0] for i in range(1,8)])
#    ypra.append([0, 0, 0])
#
#    for target in ypra:
#        arm.set_start_state_to_current_state()
#        #yaw, pitch, roll angles
#        quaternion = euler_to_quaternion(
#                math.radians(target[0]),
#                math.radians(target[1]),
#                math.radians(target[2]))
#        logger.info(f"target yaw pitch roll: {target}")
#        logger.info(f"quaternion raw values: {quaternion}")
#
#        pose_goal = PoseStamped()
#        pose_goal.header.frame_id = "link_base"
#        pose_goal.pose.orientation.w = quaternion[0]
#        pose_goal.pose.orientation.x = quaternion[1]
#        pose_goal.pose.orientation.y = quaternion[2]
#        pose_goal.pose.orientation.z = quaternion[3]
#
#        logger.info(f"pose type: {type(pose_goal.pose.orientation)}")
#
#        #these are METERS! don't push your luck
#        pose_goal.pose.position.x = 0.0
#        pose_goal.pose.position.y = 0.0
#        pose_goal.pose.position.z = 0.5
#        arm.set_goal_state(
#            # that or joint6_flange maybe?
#            pose_stamped_msg=pose_goal, pose_link="link6")
#
#        # plan to goal
#        plan_and_execute(moveit, arm, logger)
#


    ############################################################################
    ## Plan Y - Planning with Multiple Pipelines simultaneously
    ## Not tested yet
    ############################################################################

    ## set plan start state to current state
    #arm.set_start_state_to_current_state()

    ## set pose goal with PoseStamped message
    #arm.set_goal_state(configuration_name="lookup")

    ## initialise multi-pipeline plan request parameters
    #multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
    #    #moveit, ["ompl_rrtc", "pilz_lin", "chomp_planner"]
    #    moveit, ["chomp_planner"]
    #    )

    ## plan to goal
    #plan_and_execute(
    #    moveit,
    #    arm,
    #    logger,
    #    multi_plan_parameters=multi_pipeline_plan_request_params,
    #    sleep_time=8.0,
    #    )

    logger.info("WE'RE DONE!")
    time.sleep(20)
    time.sleep(20)
    time.sleep(20)
    time.sleep(20)
    time.sleep(100)


if __name__ == "__main__":
    main()
