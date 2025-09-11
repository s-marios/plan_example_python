#!/usr/bin/env python3
"""
Spawn objects with random sizes at random places for the robot to pick up.

Add option object_spawner_exec:=random_object_spawner to the launch file
arguments in order to use this.
"""

# core python libraries
import time
import math
import code
import random

# generic ros libraries
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

# ros2 messages
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive


class Spawner:

    def __init__(self):
        rclpy.init()
        self.logger = get_logger("object_spawner")

        self.node = rclpy.create_node("spawn_node")
        self.logger.info("created spawn node!")

        self.publisher = self.node.create_publisher(PlanningScene, "/planning_scene", 10)
        self.logger.info("created spawn node object publisher!")

        self.subscriber = self.node.create_subscription(
                PlanningScene,
                "/planning_scene",
                self.scene_changed_callback,
                10)

        self.i = 0

    def destroy(self):
        self.publisher.destroy()
        self.node.destroy_node()
        rclpy.shutdown()

    def create_object(
            self,
            object_id = "my_object",
            dimensions = [0.03, 0.03, 0.16],
            position = [0.2, -0.2, 0.8]) -> CollisionObject:
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world" #we're spawning things in world coordinates
        collision_object.id = object_id

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = dimensions
        collision_object.primitives.append(primitive)

        object_pose = Pose()
        object_pose.position.x = position[0]
        object_pose.position.y = position[1]
        object_pose.position.z = position[2]
        #collision_object.primitive_poses.append(object_pose)
        collision_object.pose = object_pose
        collision_object.operation = CollisionObject.ADD
        return collision_object

    def publish_object(self, obj: CollisionObject):
        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(obj)

        self.publisher.publish(planning_scene)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.logger.info("spawned an object!")

    def scene_changed_callback(self, msg: PlanningScene):
        self.logger.info("TODO: scene changed!")

        if msg.is_diff is False:
            return

        for obj in msg.world.collision_objects:
            if obj.operation == CollisionObject.REMOVE and obj.id.startswith("pick"):
                self.logger.info(f"object {obj.id} was removed from the scene!")
                self.spawn_random_object()

    def spawn_random_object(self):
        self.i = self.i + 1
        x = random.uniform(0.15, 0.30)
        y = random.uniform(-0.25,0.25)
        z = 0.08

        dimensions = [random.uniform(0.05, 0.1) for i in range(0,3)]
        pos = [x,y,dimensions[2]/2.]

        pick_object = self.create_object(
                object_id = f"pick_{self.i}",
                position = pos,
                dimensions = dimensions,
        )

        self.publish_object(pick_object)

    def start_spinning(self):
        rclpy.spin(self.node)


def main():

    ###################################################################
    # Node Setup
    ###################################################################
    # just to make sure everything is set up before we start doing things
    spawner = Spawner()

    ############################################################################
    ## Add attached virtual objects
    ############################################################################
    time.sleep(10)

    #spawner.setup_obstacles()

    # spawn one initial object to get us going
    spawner.spawn_random_object()

    # new objects will be spawned when we receive
    # object remove messsages in the subscriber callback function
    #infinte spinning here
    spawner.start_spinning()

    spawner.destroy()


if __name__ == "__main__":
    main()
