#!/usr/bin/env python3

"""
GPSR Task manager implementation using python-statemachine
"""

# Import libraries
from statemachine import State, StateMachine
import rospy

# ROS messages
from frida_hri_interfaces.msg import Command, CommandList
from frida_hri_interfaces.msg import ConversateAction, ConversateFeedback, ConversateGoal, ConversateResult

# Python submodules
from hri_tasks import TasksHRI
from manipulation_tasks import TasksManipulation
from nav_tasks import TasksNav
from vision_tasks import TasksVision

from constants.areas import Area
from constants.states import States
from constants.topics import Topics
import time

COMMANDS_TOPIC = "/task_manager/commands"

NAV_FAKE = True
MANIPULATION_FAKE = True
HRI_FAKE = False
VISION_FAKE = True


class Gpsr_SM(StateMachine):

    introduction = State(initial=True)  # Robot introduces itself
    idle = State()  # Robot is waiting for commands
    running = State()  # Robot is executing commands
    error = State()  # Robot is in error state

    introduce = introduction.to(idle)
    execute_command_transition = running.to(running) | idle.to(running)
    finished_execution = running.to(idle) | introduction.to(idle)
    error_transition = running.to(error) | idle.to(error) | error.to(error)
    exit_error = error.to(idle)

    perceived_information = ""

    timer = time.time()

    def __init__(self, task_managers):
        print("IN INIT FUNCTION")
        self.task_managers = task_managers
        super().__init__()

    def on_enter_state(self, event, state):
        self.timer = time.time()
        print(f"Entering '{state.id}' state from '{event}' event.")

    def on_enter_introduction(self, event, state):
        self.execute_command(Command(
            action="speak", complement="Hello, I'm Frida, your personal assistant. I'm here to help you with your tasks. Please tell me what you need me to do.", characteristic=True))
        self.finished_execution()

    def on_exit_state(self, event, state):
        print(f"Time spent in state {state.id}:", time.time() - self.timer)
        print(f"Exiting '{state.id}' state from '{event}' event.")

    def on_enter_running(self, event, state, message: Command):
        print("ENTER RUNNING")
        print(f"Executing command: {message.action} -> {message.complement}")
        self.execute_command(message)
        self.finished_execution()

    def execute_command(self, command: Command):
        task_result = 0
        found = False
        for sub_task_manager in self.task_managers:
            if command.action in sub_task_manager.get_tasks():

                task_result = sub_task_manager.execute_command(
                    command.action, command.complement, command.characteristic
                )
                found = True
                break

        if not found:
            rospy.logerr(f"Task not found: {command.action}")
            self.perceived_information += f"Task not found: {command.action} "
        else:
            self.perceived_information += f"{command.action} {command.complement} {task_result} "


class Gpsr_ROS:
    def __init__(self) -> None:
        self._node = rospy.init_node("gpsr_server")
        self._rate = rospy.Rate(200)
        self._sub = rospy.Subscriber(
            Topics.COMMANDS.value, CommandList, self.commands_callback)
        self.command_queue = CommandList()
        self.command_queue.commands = []

        task_managers = [
            TasksNav(NAV_FAKE),
            TasksManipulation(MANIPULATION_FAKE),
            TasksVision(VISION_FAKE),
            TasksHRI(HRI_FAKE)
        ]

        self.sm = Gpsr_SM(task_managers=task_managers)

        rospy.loginfo("GPSR Engine initialized")

    def commands_callback(self, commands_input: CommandList) -> None:
        """Receive processed commands from the interpreter and call executions from the queue"""

        if commands_input:
            rospy.loginfo("Received commands")
        else:
            rospy.loginfo("No commands received")
            return

        for command in commands_input.commands:
            rospy.loginfo(
                f"Added command to queue: {command.action} -> {command.complement} : {command.characteristic}")
            self.command_queue.commands.append(command)

    def run(self) -> None:
        """Run the task manager"""
        while not rospy.is_shutdown():
            if self.sm.current_state == Gpsr_SM.introduction:
                self.sm.introduce()
            elif self.sm.current_state == Gpsr_SM.idle:
                if len(self.command_queue.commands) > 0:
                    self.sm.send("execute_command_transition",
                                 message=self.command_queue.commands[0])
                    self.command_queue.commands.pop(0)
                # else:
                #     self.sm.finished_tasks()

            self._rate.sleep()

    def cancel_commands(self) -> None:
        """Method to cancel the current command"""
        rospy.loginfo("Canceled current commands")
        raise NotImplementedError()


if __name__ == "__main__":
    task_manager = Gpsr_ROS()
    task_manager.run()


"""
Test

rostopic pub /task_manager/commands frida_hri_interfaces/CommandList "commands:
- action: 'speak'
  characteristic: 'This is a test'
  complement: 'False'" 

"""
