#!/usr/bin/env python3

"""
This script manages the implementation of each HRI tasks
"""

### Import libraries
import rospy
import actionlib
from enum import Enum

### ROS messages
from std_msgs.msg import String
from frida_hri_interfaces.srv import Speak
from frida_hri_interfaces.srv import ExtractInfo

from tasks_base import TasksBase
from constants.services import Services
from constants.servers import Server
from constants.topics import Topics


class TasksHRI(TasksBase):
    class Tasks(Enum):
        say = "say"
        say_async = "say_async"
        extract_data = "extract_data"
        convesation = "conversation"

    def __init__(self) -> None:
        self.pub_speak = rospy.Publisher(Topics.SPEAK_NOW.value, String, queue_size=10)

        rospy.wait_for_service(Services.SPEAK.value, timeout=5.0)
        rospy.wait_for_service(Services.EXTRACT_DATA.value, timeout=5.0)

        self.speak_client = rospy.ServiceProxy(Services.SPEAK.value, Speak)
        self.extract_data_client = rospy.ServiceProxy(Services.EXTRACT_DATA.value, ExtractInfo)
        rospy.loginfo("HRI Task Manager initialized")

    def execute_command(self, action: str, characteristic: str, complement: str) -> int:
        """Method to execute each command"""

        rospy.loginfo(f"HRI Command: {action} | {characteristic} | {complement}")
        
        if action not in self.Tasks.__members__:
            rospy.logerr(f"Action {action} not found")
            return 0

        if action == self.Tasks.say:
            self.speak(complement, wait=True)
        elif action == self.Tasks.say_async:
            self.speak(complement, wait=False)
        elif action == self.Tasks.extract_data:
            self.extract_data(full_text=complement, data=characteristic)
        elif action == self.Tasks.convesation:
            pass
        return 1

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        self.conversation_client.cancel_all_goals()
        rospy.loginfo("Command canceled HRI")

    def extract_data(self, full_text: str, data: str) -> None:
        """Method to extract information from the given text
        Args:
            full_text (str): The complete text
            data (str): The information to extract
        """
        rospy.loginfo(f"Extracting {data} from {full_text}.")

        return self.extract_data_client(full_text=full_text, data=data).result

    def speak(self, text: str, wait: bool = True) -> None:
        """Method to publish directly text to the speech node"""
        if wait:
            self.speak_client(text)
        else:
            self.pub_speak.publish(text)

if __name__ == "__main__":
    try:
        rospy.init_node("hri_task_manager")
        t = TasksHRI()
        
        print("result:", t.extract_data("I'm Oscar, and my favorite drink is Fanta", "drink"))
        rospy.spin()
                  
    except rospy.ROSInterruptException as e:
        rospy.logerr("Error: {}".format(e))
        pass