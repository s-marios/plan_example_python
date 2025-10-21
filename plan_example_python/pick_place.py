#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

# core python libraries
import time
import copy
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

    def move_to(self, obj_pose: Pose):
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

        self.arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link=self.pose_link)
        self.plan_and_execute()



class RobotWithoutGripper(RobotLogic):

    def __init__(self, moveit, logger, pose_link):
         super().__init__(moveit, logger, pose_link)
         self.arm = self.moveit.get_planning_component("lite6")

    def move_to_object(self, obj_pose: Pose):
         self.move_to(obj_pose)

    def move_to_drop_location(self, location: Pose):
        self.move_to(location)

class RobotWithVacuumGripper(RobotLogic):

    def __init__(self, moveit, logger, srv_cli):
         super().__init__(moveit, logger, "link_eef")
         self.cli = srv_cli
         self.arm = self.moveit.get_planning_component("lite6")


    def attach_physical_object(self, obj):
        req = VacuumGripperCtrl.Request()
        req.on = True
        req.wait = True
        res = self.cli.call(req)
        #TODO: check result

    def detach_physical_object(self):
        req = VacuumGripperCtrl.Request()
        req.on = False
        req.wait = True
        res = self.cli.call(req)

    def move_to_object(self, obj_pose: Pose):
         self.move_to(obj_pose)

    def move_to_drop_location(self, location: Pose):
        self.move_to(location)


class PickAndPlace():

    def __init__(self):
        rclpy.init()
        self.logger = get_logger("pick_place")

        # instantiate MoveItPy instance and get planning component
        self.moveit = MoveItPy(node_name="pick_place")
        #self.arm = self.moveit.get_planning_component("lite6")
        self.logger.info("Pick and Place MoveItPy instance created")

        ### Setup a second node
        self.node = rclpy.create_node("object_despawner")
        self.logger.info("created object despawn node!")

        ## Setup necessary pub/sub
        self.publisher = self.node.create_publisher(PlanningScene, "/planning_scene", 10)
        self.subscriber = self.node.create_subscription(
                PlanningScene,
                "/planning_scene",
                self.scene_changed_callback,
                10)

        self.object_queue = SimpleQueue()

        ## Setup necessary parameters
        ## vacuum gripper
        self.node.declare_parameter("add_vacuum_gripper", False)
        self.vacuum_gripper = self.node.get_parameter("add_vacuum_gripper").get_parameter_value().bool_value

        ## robot_ip
        self.node.declare_parameter("robot_ip", "fake")
        self.robot_ip = self.node.get_parameter("robot_ip").get_parameter_value().string_value

        self.logger.info(f"CONFIGURATION: vacuum_gripper {self.vacuum_gripper}, robot_ip {self.robot_ip}")

        # finally initialize appropriate robot logic
        # TODO: check parameters and instantiate appropriate classes
        if self.vacuum_gripper and self.robot_ip != "fake":
            self.logger.info(f"this is real arm, with vacuum gripper")
            srv_cli = self.node.create_client(VacuumGripperCtrl, "/ufactory/set_vacuum_gripper")
            while not srv_cli.wait_for_service(timeout_sec=1.0):
                self.logger.info(f"waiting for xarm VacuumGripperCtrl to come up")
            self.robot_logic = RobotWithVacuumGripper(self.moveit, self.logger, srv_cli)
        elif self.vacuum_gripper and self.robot_ip == "fake":
            self.logger.info(f"we have a fake robot, but with a vacuum gripper")
            self.robot_logic = RobotWithoutGripper(self.moveit, self.logger, "link_eef")
        else:
            #we have a physical or fake robot without vacuum gripper
            self.robot_logic = RobotWithoutGripper(self.moveit, self.logger, "link6")



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

    def attach(self, obj: CollisionObject) -> AttachedCollisionObject:
        self.logger.info(f"ATTACHING OBJECT ID: {obj.id}")


        # TODO change link6 depending on the parameters
        obj.header.frame_id = "link6" #relative to the eef

        object_pose = Pose()
        object_pose.position.x = 0.0
        object_pose.position.y = 0.0
        #TODO this is bad, because we only deal with boxes
        #change this to deal with other primitives such as cylinders
        object_pose.position.z = obj.primitives[0].dimensions[2] / 2.0

        obj.pose = object_pose

        attached_object = AttachedCollisionObject()
        attached_object.object = obj
        attached_object.link_name = "link6"
        attached_object.object.operation = CollisionObject.ADD
        attached_object.touch_links = ["link6"]

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        planning_scene.robot_state.is_diff = True
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

            #step 2: move to object (TODO Implicit assumption that all objects are boxes..)
            pick_location = copy.copy(obj.pose)
            pick_location.position.z += obj.primitives[0].dimensions[SolidPrimitive.BOX_Z]/2.
            self.robot_logic.move_to_object(pick_location)
            #self.move_to(pick_location)

            #step 3: pickup/attach object
            attached_object = self.attach(obj)
            self.robot_logic.attach_physical_object(obj)

            #step 4: move to drop location
            drop_location.position.z = obj.primitives[0].dimensions[SolidPrimitive.BOX_Z] + 0.01
            #self.move_to(drop_location)
            self.robot_logic.move_to_drop_location(drop_location)

            #step 5: detach
            self.detach(attached_object)
            self.robot_logic.detach_physical_object()


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
