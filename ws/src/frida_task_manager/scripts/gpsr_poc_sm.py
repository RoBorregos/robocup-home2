#!/usr/bin/env python3

"""
GPSR Task manager implementation using python-statemachine
"""

### Import libraries
from statemachine import State, StateMachine
import rospy

### ROS messages
from frida_hri_interfaces.msg import Command, CommandList
from frida_hri_interfaces.msg import ConversateAction, ConversateFeedback, ConversateGoal, ConversateResult

### Python submodules
from hri_tasks import TasksHRI
from manipulation_tasks import TasksManipulation
from nav_tasks import TasksNav
from vision_tasks import TasksVision

from constants.areas import Area
from constants.states import States
from constants.topics import Topics

class Gpsr_SM(StateMachine):
    
    introduction = State(initial=True) # Robot introduces itself
    idle = State() # Robot is waiting for commands
    running = State() # Robot is executing commands
    
    introduce = introduction.to(idle)
    execute_command = running.to(running) | idle.to(running)
    finished_execution = running.to(idle)
    
    def __init__(self, task_managers):
        super().__init__()
        self.task_managers = task_managers

    def on_enter_state(self, event, state):
        print(f"Entering '{state.id}' state from '{event}' event.")
    
    def on_exit_state(self, event, state):
        print(f"Exiting '{state.id}' state from '{event}' event.")

class Gpsr_ROS:
    def __init__(self) -> None:
        self._node = rospy.init_node("gpsr_server")
        self._rate = rospy.Rate(200)
        self._sub = rospy.Subscriber(Topics.COMMANDS, CommandList, self.commands_callback)
        self.command_queue = []
        self.command_queue = CommandList()
        self.command_queue.commands = []
        
        task_managers = [
            TasksManipulation(),
            TasksNav(),
            TasksVision(),
            TasksHRI()
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
            rospy.loginfo(f"Added command to queue: {command.action} -> {command.complement} : {command.characteristic}")
            self.command_list.commands.append(command)

        # self.subtask_manager["hri"].speak("I've understood your request, I'll execute the following commands:")
        # for command in commands_input.commands:
        #     command_clean =  command.characteristic + " " + command.complement.replace("_", " ")
        #     if command.action == 'interact':
        #         self.subtask_manager["hri"].speak(f"I'll interact with you based on this request: {command_clean}", wait=True)
        #     else:
        #         self.subtask_manager["hri"].speak(f"I'll {command.action} {command_clean}", wait=True)
        # self.subtask_manager["hri"].speak("Let's start")
        
        # self.current_queue = commands_input.commands
        # self.past_state = self.current_state
    
    def run(self) -> None:
        """Run the task manager"""
        while not rospy.is_shutdown():
            if self.sm.current_state == Gpsr_SM.introduction:
                self.sm.introduce()
            elif self.sm.current_state == Gpsr_SM.idle:
                if self.command_queue:
                    self.sm.execute_command()
                else:
                    self.sm.finished_tasks()
            
            
            self._rate.sleep(10)
    
    def cancel_commands(self) -> None:
        """Method to cancel the current command"""
        rospy.loginfo("Canceled current commands")
        raise NotImplementedError()

if __name__ == "__main__":
    sm = Gpsr_SM(task_managers=[])
    img_path = sm._graph().write_png("gpsr.png") 
    
    # task_manager = Gpsr_ROS()
    # task_manager.run()
