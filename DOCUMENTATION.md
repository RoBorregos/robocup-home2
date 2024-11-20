# Documentation of Current State 2024

## Overview

This document provides a detailed overview of the purpose of each area in the service robot and the technologies used. It includes examples of each ROS node's purpose and a detailed example of a task flow for the `receptionist_task_manager.py` node.

## Purpose of Each Area

### Human-Robot Interaction (HRI)
The HRI area focuses on enabling the robot to interact effectively with humans. It includes tasks such as speaking, guest information retrieval, and guest analysis. The `hri_tasks.py` node manages the implementation of HRI tasks.

### Manipulation
The manipulation area focuses on enabling the robot to perform tasks that involve manipulating objects. It includes tasks such as picking, placing, and moving arm joints. The `manipulation_tasks.py` node manages the implementation of manipulation tasks.

### Navigation
The navigation area focuses on enabling the robot to move to specific locations and store the current location. The `nav_tasks.py` node manages the implementation of navigation tasks.

### Vision
The vision area focuses on enabling the robot to process visual information and interact with its environment. It includes tasks such as saving face names, checking for persons, and finding free seats. The `vision_tasks.py` node manages the implementation of vision tasks.

## Examples of ROS Nodes Purpose

* `receptionist_task_manager.py` (`ws/src/frida_task_manager/scripts/receptionist_task_manager.py`): Manages the receptionist task, including guest interaction, face recognition, and navigation to the living room.
* `breakfast_task_manager.py` (`ws/src/frida_task_manager/scripts/breakfast_task_manager.py`): Manages the breakfast task, including picking and placing objects, pouring, and navigating to different locations.
* `demo_tmr24.py` (`ws/src/frida_task_manager/scripts/demo_tmr24.py`): Demonstrates the task manager for the breakfast task, including guest interaction, face recognition, and serving breakfast.
* `task_manager_server.py` (`ws/src/frida_task_manager/scripts/task_manager_server.py`): Serves as a template for developing multiple task managers for various tasks in RoboCup@Home.
* `hri_tasks.py` (`ws/src/frida_task_manager/scripts/hri_tasks.py`): Manages the implementation of Human-Robot Interaction (HRI) tasks, including speaking, guest information retrieval, and guest analysis.
* `manipulation_tasks.py` (`ws/src/frida_task_manager/scripts/manipulation_tasks.py`): Manages the implementation of manipulation tasks, including picking, placing, and moving arm joints.
* `nav_tasks.py` (`ws/src/frida_task_manager/scripts/nav_tasks.py`): Manages the implementation of navigation tasks, including moving to specific locations and storing the current location.
* `vision_tasks.py` (`ws/src/frida_task_manager/scripts/vision_tasks.py`): Manages the implementation of vision tasks, including saving face names, checking for persons, and finding free seats.

## Detailed Example of a Task Flow for `receptionist_task_manager.py`

Here is a detailed example of a task flow for the `receptionist_task_manager.py` node:

* The `ReceptionistTaskManager` class is initialized, setting up the ROS node, rate, and subscribers. It also initializes the subtask managers for different areas such as HRI, manipulation, navigation, and vision.
* The `run` method is the main loop for the task manager, which continuously checks the current state and executes the corresponding actions.
* The task flow starts with the `WAITING_GUEST` state, where the robot waits for a guest to be detected. It uses the vision subtask manager to check for a person and follows the detected face using the manipulation subtask manager.
* Once a guest is detected, the state transitions to `SELF_INTRODUCTION`, where the robot introduces itself and asks for the guest's name and favorite drink using the HRI subtask manager.
* The state then transitions to `REQUEST_GUEST_INFORMATION`, where the robot retrieves the guest's information and stores it in the `Guest` class.
* In the `SAVE_USER_FACE` state, the robot saves the guest's face and analyzes it using the vision and HRI subtask managers.
* The robot then navigates to the living room in the `GO_TO_LIVING_ROOM` state using the navigation subtask manager.
* In the `INTRODUCE_GUEST_1` and `INTRODUCE_GUEST_2` states, the robot introduces the guest to the host and other guests using the HRI and manipulation subtask managers.
* The robot then gazes at the current guest in the `GAZE_AT_GUEST` state and informs them about finding a free seat.
* In the `FIND_FREE_SEAT` state, the robot uses the vision subtask manager to find a free seat and directs the guest to it using the manipulation subtask manager.
* The robot waits for the guest to sit in the `WAIT_USER_TO_SIT` state and then transitions to the `GO_TO_ENTRANCE` state to return to the entrance.
* The task flow continues until all guests are processed, and the robot transitions to the `SHUTDOWN` state, indicating the end of the task.

This detailed example demonstrates how the `receptionist_task_manager.py` node manages the receptionist task by coordinating various subtask managers and transitioning through different states to achieve the desired behavior. The code for this task manager can be found in `ws/src/frida_task_manager/scripts/receptionist_task_manager.py`. The subtask managers for HRI, manipulation, navigation, and vision are implemented in `ws/src/frida_task_manager/scripts/hri_tasks.py`, `ws/src/frida_task_manager/scripts/manipulation_tasks.py`, `ws/src/frida_task_manager/scripts/nav_tasks.py`, and `ws/src/frida_task_manager/scripts/vision_tasks.py` respectively.
