#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

# core python libraries
import time
import math
import code
import threading
from queue import SimpleQueue

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
from geometry_msgs.msg import Pose, PoseStamped
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

class PickAndPlace():

    def __init__(self):
        rclpy.init()
        self.logger = get_logger("pick_place")

        # instantiate MoveItPy instance and get planning component
        self.moveit = MoveItPy(node_name="pick_place")
        self.arm = self.moveit.get_planning_component("lite6")
        self.logger.info("Pick and Place MoveItPy instance created")

        ### Setup a second node and a publisher
        self.node = rclpy.create_node("object_despawner")
        self.logger.info("created object despawn node!")

        self.publisher = self.node.create_publisher(PlanningScene, "/planning_scene", 10)
        self.subscriber = self.node.create_subscription(
                PlanningScene,
                "/planning_scene",
                self.scene_changed_callback,
                10)

        self.object_queue = SimpleQueue()

    def start_spinning(self):
        self.thread = threading.Thread(target=self.spin)
        self.thread.start()

    def spin(self):
        self.logger.info("sub_thread: start spinning!")
        rclpy.spin(self.node)


    def scene_changed_callback(self, msg: PlanningScene):
        self.logger.info("scene changed!")
        for obj in extract_objects_from_scene_message(msg):
            self.object_queue.put(obj)

    def move_to(self):
        pass

    def attach(self):
        pass

    def detach(self):
        pass

def extract_objects_from_scene_message(scene: PlanningScene) -> list[CollisionObject]:
    result = []
    if scene.is_diff != True:
        return result


    for obj in scene.world.collision_objects:
        if obj.id.startswith("pick") and obj.operation == CollisionObject.ADD:
            result.append(obj)

    return result

def main():

    #############################################################################
    ### Plan 6 - Attach virtual objects, test raw moveit messages
    #############################################################################
    pick_place = PickAndPlace()
    pick_place.start_spinning()

    while True:
        #step 1: detect new object (WIP)
        pick_place.logger.info(f"we got a message from the queue!!!!")
        obj = pick_place.object_queue.get()

        #step 2: move to object
        pick_place.logger.info(f"object coordinates: {
                               obj.pose.position.x,
                               obj.pose.position.y,
                               obj.pose.position.z}")


        pick_place.arm.set_start_state_to_current_state()
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "world"

        q = euler_to_quaternion(math.radians(0), math.radians(180), math.radians(0))

        pose_goal.pose.orientation.w = q[0]
        pose_goal.pose.orientation.x = q[1]
        pose_goal.pose.orientation.y = q[2]
        pose_goal.pose.orientation.z = q[3]


        pose_goal.pose.position.x = obj.pose.position.x
        pose_goal.pose.position.y = obj.pose.position.y
        pose_goal.pose.position.z = obj.pose.position.z * 2.1

        #pick_place.logger.info(f"Moving to point: x={p[0]}, y={p[1]}, z={p[2]}")
        pick_place.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")
        plan_and_execute(pick_place.moveit, pick_place.arm, pick_place.logger, sleep_time=0.5)

        #step 3: pickup/attach object

        #step 4: move to drop location

        #step 5: detach

    #for i in range(0, 10):
    #    pick_place.logger.info(f"THIS IS THE MAIN THREAD!!!! {i}")
    #    time.sleep(10)


    #pspub = node.create_publisher(PlanningScene, "/planning_scene", 10)

    #collision_object = CollisionObject()
    #collision_object.header.frame_id = "link6" #relative to the eef
    #collision_object.id = "my_object"

    #primitive = SolidPrimitive()
    #primitive.type = SolidPrimitive.BOX
    #primitive.dimensions = [0.03, 0.03, 0.16]
    #collision_object.primitives.append(primitive)

    #object_pose = Pose()
    #object_pose.position.x = 0.0
    #object_pose.position.y = 0.0
    #object_pose.position.z = 0.08
    #collision_object.primitive_poses.append(object_pose)

    #attached_object = AttachedCollisionObject()
    #attached_object.object = collision_object
    #attached_object.link_name = "link6"
    #attached_object.object.operation = CollisionObject.ADD
    #attached_object.touch_links = ["link6"]

    #planning_scene = PlanningScene()
    #planning_scene.is_diff = True
    #planning_scene.robot_state.attached_collision_objects.append(attached_object)
    #pspub.publish(planning_scene)
    #rclpy.spin_once(node, timeout_sec=1.0)

    ###OR EQUIVALENTLY
    ##planning_scene_monitor = moveit.get_planning_scene_monitor()
    ##with planning_scene_monitor.read_write() as scene:
    ##    scene.process_attached_collision_object(attached_object)
    ##    scene.current_state.update()



    ############################################################################
    ## Plan 3.1 - set goal state with PoseStamped message
    ## points on a cube, effector orientaion pointing "outwards" from the
    ## (roughly) center of the cube
    ############################################################################

    ## set pose goal with PoseStamped message
    #from geometry_msgs.msg import PoseStamped

    #angles = [
    #    [225, 135, 0],
    #    [315, 135, 0],
    #    #[45, 135, 0],
    #    #[135, 135, 0],
    #    #[135, 45, 0],
    #    #[45, 45, 0],
    #    #[315, 45, 0],
    #    #[225, 45, 0],
    #        ]

    #quaternions = [euler_to_quaternion(
    #    math.radians(angle[0]),
    #    math.radians(angle[1]),
    #    math.radians(angle[2])
    #    ) for angle in angles]

    #cube = [
    #    [-0.25, -0.25, 0.25],
    #    [0.25, -0.25, 0.25],
    #    #[0.25, 0.25, 0.25],
    #    #[-0.25, 0.25, 0.25],
    #    #[-0.25, 0.25, 0.6],
    #    #[0.25, 0.25, 0.6],
    #    #[0.25, -0.25, 0.6],
    #    #[-0.25, -0.25, 0.6],
    #        ]

    ## Move through all waypoints
    #for i, (p, q) in enumerate(zip(cube, quaternions), start=1):
    #    arm.set_start_state_to_current_state()
    #    pose_goal = PoseStamped()
    #    pose_goal.header.frame_id = "link_base"
    #    pose_goal.pose.orientation.w = q[0]
    #    pose_goal.pose.orientation.x = q[1]
    #    pose_goal.pose.orientation.y = q[2]
    #    pose_goal.pose.orientation.z = q[3]

    #    pose_goal.pose.position.x = p[0]
    #    pose_goal.pose.position.y = p[1]
    #    pose_goal.pose.position.z = p[2]

    #    logger.info(f"Moving to point {i}: x={p[0]}, y={p[1]}, z={p[2]}")
    #    arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")
    #    plan_and_execute(moveit, arm, logger, sleep_time=0.5)

    #############################################################################
    ### Plan 7 - Remove attached virtual object
    #############################################################################
    #prev_scene = None
    #with planning_scene_monitor.read_only() as scene:
    #    prev_scene = scene.planning_scene_message


    ## remove the lightsaber
    #planning_scene = PlanningScene()
    #planning_scene.is_diff = True
    #attached_object.object.operation = CollisionObject.REMOVE
    ### detatch object
    #planning_scene.robot_state.attached_collision_objects.append(attached_object)
    #planning_scene.robot_state.is_diff = True
    ### remove it from the world completely
    ## you can comment out the line below and see that the object is still in the world
    #planning_scene.world.collision_objects.append(attached_object.object)
    #pspub.publish(planning_scene)
    #rclpy.spin_once(node, timeout_sec=1.0)


    #time.sleep(10)
    ## houdini, make the lightsaber appear again!
    #pspub.publish(prev_scene)
    #rclpy.spin_once(node, timeout_sec=1.0)

    #time.sleep(3)

    ## remove the green boxes
    #planning_scene_monitor = moveit.get_planning_scene_monitor()
    #with planning_scene_monitor.read_write() as scene:
    #    green_boxes.operation = CollisionObject.REMOVE
    #    scene.apply_collision_object(green_boxes)
    #    scene.current_state.update()

    #    ## this did not the way I expected it to
    #    ## not exactly sure that the attached object gets removed
    #    #scene.process_attached_collision_object(attached_object)
    #    #scene.current_state.update()

    ## this didn't seem to work either..
    #    robot_state.clear_attached_bodies()
    #    robot_state.update()


    print("WE'RE DONE!")
    time.sleep(20)


if __name__ == "__main__":
    main()
