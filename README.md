# Plan Example Python

This is a sample project demonstrating the use of `MoveItPy` with the [Ufactory Lite 6 robotic arm](https://www.ufactory.us/product/lite-6).

# Installation

First, create a ROS2 workspace. Then, clone this repository in the `src` folder.

This project relies on [`xarm_ros2` project ](https://github.com/xArm-Developer/xarm_ros2) and its ROS2 packages to work. As such, you can:

* prepare an underlay with `xarm_ros2` already compiled
* clone `xarm_ros2` in the `src` folder of your workspace and build both projects at the same time.

Assuming that you have both projects cloned into your workspace, proceed to build the software using:

```
colcon build
```

Please make sure you have fulfilled the prerequisites for the `xarm_ros2` project.

# Usage

## Sourcing the appropriate ROS2 overlay

After you have successfully built the project, source the appropriate overlay:

```
source PROJECT_WS/install/setup.bash
```

where `PROJECT_WS` is your ROS2 workspace.

## Running the moveit example

```
ros2 launch plan_example_python motion_planning_python_api_tutorial.launch.py
```

## Running the pick and place task


```
 ros2 launch plan_example_python vpp.launch.py
```

Information regarding launch arguments can be seen by doing:

```
 ros2 launch plan_example_python vpp.launch.py --show-args
```

## Specifying a robot

Use the `robot_ip:=ROBOT_IP` argument to connect to a real robot. If this argument is omitted, a virtual robot will be used instead.

## Adding a vacuum gripper

To add a vacuum gripper, specify `add_vacuum_gripper:=true` as an argument. It works both with a virtual and a real robot.

For a real gripper, we rely on the services provided by `xarm_api`. You need to enable the `set_vacuum_gripper` service. The simplest way is to:

* copy the `xarm_ros2/xarm_api/config/xarm_params.yaml` to `xarm_ros2/xarm_api/config/xarm_user_params.yaml`
* set the `set_vacuum_gripper` service to `true`

For more details, read the `xarm_api` documentation.

## Unity Branch

Ensure that you have the package [demo_planning_msgs](https://github.com/s-marios/demo_planning_msgs) in your workspace. 

After building your workspace, you can launch the `moveit` planning service as follows:

```
ros2 launch plan_example_python planning_service.launch.py
```

# 3RD PARTY SOFTWARE

This projects makes use of third party software, the details of which can be found in the [LICENSE_3RD_PARTY](./LICENSE_3RD_PARTY) file.
