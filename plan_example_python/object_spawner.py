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

    def destroy(self):
        self.publisher.destroy()
        self.node.destroy_node()
        rclpy.shutdown()

    def create_object(
            self,
            object_id = "my_object",
            dimensions = [0.03, 0.03, 0.16],
            position = [0.2, -0.2, 0.08]) -> CollisionObject:
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
        #planning_scene.name = 'vpp'
        #planning_scene.robot_model_name = 'UF_ROBOT'
        planning_scene.world.collision_objects.append(obj)

        self.publisher.publish(planning_scene)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.logger.info("spawned an object!")


def main():

    ###################################################################
    # Node Setup
    ###################################################################
    # just to make sure everything is set up before we start doing things
    time.sleep(5)
    spawner = Spawner()

    ############################################################################
    ## Add attached virtual objects
    ############################################################################
    time.sleep(5)

    for i in range(0, 10):
        pos = [0.2, -0.2 + (0.4 * i / 10.0 ),  0.08]
        pickup_object = spawner.create_object(object_id=f"pick_{i}", position=pos)
        spawner.publish_object(pickup_object)
        time.sleep(5)

    print("WE'RE DONE!")
    spawner.destroy()


if __name__ == "__main__":
    main()
