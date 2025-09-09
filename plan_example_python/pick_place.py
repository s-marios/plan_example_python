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

    def move_to(self, obj_pose: Pose, height_compensate=False):
        self.logger.info(f"object coordinates: {
                               obj_pose.position.x,
                               obj_pose.position.y,
                               obj_pose.position.z}")

        self.arm.set_start_state_to_current_state()
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "world"

        q = euler_to_quaternion(math.radians(0), math.radians(180), math.radians(0))

        pose_goal.pose.orientation.w = q[0]
        pose_goal.pose.orientation.x = q[1]
        pose_goal.pose.orientation.y = q[2]
        pose_goal.pose.orientation.z = q[3]

        pose_goal.pose.position.x = obj_pose.position.x
        pose_goal.pose.position.y = obj_pose.position.y
        pose_goal.pose.position.z = obj_pose.position.z

        if height_compensate == True:
            pose_goal.pose.position.z = obj_pose.position.z * 2.1


        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link6")
        self.plan_and_execute()

    def attach(self, obj: CollisionObject) -> AttachedCollisionObject:
        self.logger.info(f"ATTACHING OBJECT ID: {obj.id}")

        obj.header.frame_id = "link6" #relative to the eef

        object_pose = Pose()
        object_pose.position.x = 0.0
        object_pose.position.y = 0.0
        object_pose.position.z = 0.08

        obj.pose = object_pose

        attached_object = AttachedCollisionObject()
        attached_object.object = obj
        attached_object.link_name = "link6"
        attached_object.object.operation = CollisionObject.ADD
        attached_object.touch_links = ["link6"]

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        self.publisher.publish(planning_scene)
        return attached_object

    def detach(self, attached_object: AttachedCollisionObject):
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
        self.publisher.publish(planning_scene)

    def main_loop(self):

        #hard-coded drop location
        drop_location = Pose()
        #x, y hard-coded
        drop_location.position.x = -0.2
        drop_location.position.y = 0.2
        #z changes depending on object height

        while True:
            #step 1: detect new object (WIP)
            self.logger.info(f"we got a message from the queue!!!!")
            obj = self.object_queue.get()

            #step 2: move to object
            self.move_to(obj.pose, True)

            #step 3: pickup/attach object
            attached_object = self.attach(obj)

            #step 4: move to drop location
            drop_location.position.z = obj.primitives[0].dimensions[SolidPrimitive.BOX_Z] + 0.01
            self.move_to(drop_location)

            #step 5: detach
            self.detach(attached_object)

    def plan_and_execute(
            self,
            single_plan_parameters=None,
            multi_plan_parameters=None,
            sleep_time=0.0,
            ):
        """Helper function to plan and execute a motion."""
        # plan to goal
        self.logger.info("Planning trajectory")
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
    pick_place.main_loop()



if __name__ == "__main__":
    main()
