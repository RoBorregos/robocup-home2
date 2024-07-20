#!/usr/bin/env python3

"""
This script manages the implementation of each HRI tasks
"""

### Import libraries
import rospy
import actionlib

### ROS messages
from std_msgs.msg import String, Bool
from frida_hri_interfaces.msg import ConversateAction, ConversateFeedback, ConversateGoal, ConversateResult
from frida_hri_interfaces.srv import Speak
from frida_hri_interfaces.srv import ItemsCategory
from frida_hri_interfaces.msg import GuestAnalysisAction, GuestAnalysisFeedback, GuestAnalysisGoal, GuestAnalysisResult
from frida_hri_interfaces.srv import GuestInfo, GuestInfoResponse
import random

SPEAK_TOPIC = "/speech/speak_now"
CONVERSATION_SERVER = "/conversation_as"
GUEST_INFO_SERVICE = "/guest_info"
GUEST_ANALYSIS_SERVER = "/guest_analysis_as"
ITEMS_CATEGORY_SERVER = "/items_category"
OBJECT_CATEGORY_SERVER = "/object_category"
KEYWORD_TOPIC = "keyword_detected"

class TasksHRI:
    STATE_ENUM = {
        "IDLE": 0,
        "RECEIVE_COMMANDS": 1,
        "EXECUTING_COMMANDS": 2,
        "STOPPING": 3,
        "ERROR": 4,
        "SHUTDOWN": 5
    }

    AREA_TASKS = ["ask", "interact", "feedback", "analyze_objects", "speak"]
    OBJECT_CATEGORIES = ["decorations", "cleaning_supplies", "toys", "fruits", "drinks", "snaks", "dishes", "food"]

    def __init__(self, fake = False) -> None:
        self.FAKE_TASK = fake
        self.fake_index = 0
        self.guest_description = ["", "", ""]

        if not self.FAKE_TASK:
            self.conversation_client = actionlib.SimpleActionClient(CONVERSATION_SERVER, ConversateAction)
            self.pub_speak = rospy.Publisher(SPEAK_TOPIC, String, queue_size=10)
            
            rospy.loginfo("[INFO] Waiting for conversation server")
            # rospy.wait_for_service(SPEAK_TOPIC, timeout=10.0)
            #self.speak_client = rospy.ServiceProxy(SPEAK_TOPIC, Speak)
            rospy.loginfo("[SUCCESS] Conversation server initialized")
            
            rospy.loginfo("[INFO] Waiting for items category server")
            try:
                rospy.wait_for_service(ITEMS_CATEGORY_SERVER, timeout=2.0)
                self.items_category_client = rospy.ServiceProxy(ITEMS_CATEGORY_SERVER, ItemsCategory)
            except rospy.ROSException as e:
                rospy.logwarn("[WARNING] Items category service not available")
            
            rospy.loginfo("[INFO] Waiting for object category server")
            try:
                rospy.wait_for_service(OBJECT_CATEGORY_SERVER, timeout=2.0)
                self.object_category_client = rospy.ServiceProxy(OBJECT_CATEGORY_SERVER, ItemsCategory)
            except rospy.ROSException as e:
                rospy.logwarn("[WARNING] Object category service not available")

        else:
            rospy.loginfo("[INFO] Fake HRI Task Manager initialized")
        
        rospy.loginfo("[SUCCESS] HRI Task Manager initialized")
        self.keyword_sub = rospy.Subscriber("keyword_detected", Bool, self.callbackKeyword)
        self.keyword_detected = False

    def execute_command(self, command: str, complement: str, perceived_information: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("[INFO] HRI Command")
        composed_request = f"{command}: {complement}, perceived info: {perceived_information}"

        if command == "analyze_guest":
            return self.analyze_guest(int(complement))
        elif command == "get_guest_info":
            return self.get_guest_info(int(complement))
        elif command == "get_guest_description":
            return self.get_guest_description(int(complement))
        elif command == "speak":
            return self.speak(complement)
        elif command == "cancel":
            return self.cancel_command()
        elif command == "analyze_objects":
            return self.get_objects_category(perceived_information.split(","))
        

        goal = ConversateGoal()
        goal.request = composed_request
        goal.wait = 0 if command == "feedback" else 1

        if not self.FAKE_TASK:
            self.conversation_client.send_goal(goal)
            if goal.wait:
                self.conversation_client.wait_for_result()
                result = self.conversation_client.get_result()
                #rospy.loginfo(f"Result: {result.success}")
                return result.success
        return 1

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        self.conversation_client.cancel_all_goals()
        rospy.loginfo("[INFO] Command canceled HRI")

    def get_guest_info(self, guest_id: int):
        """Method to get the guest information
        Returns the name and favorite drink of the guest"""
        if self.FAKE_TASK:
            return "sample_name", "sample_drink"
        
        rospy.wait_for_service("/guest_info")
        try:
            guest_info = rospy.ServiceProxy(GUEST_INFO_SERVICE, GuestInfo)
            response = guest_info(guest_id)
            if response.success:
                return response.name, response.favorite_drink
            return "error", "error"
        except rospy.ServiceException:
            rospy.logerr("Service call failed")
            return "error", "error"
        
    def analyze_guest(self, guest_id: int) -> str:
        """Method to analyze the guest
        Returns the guest name and favorite drink"""
        client = actionlib.SimpleActionClient(GUEST_ANALYSIS_SERVER, GuestAnalysisAction)
        client.wait_for_server()

        goal = GuestAnalysisGoal()
        goal.guest_id = guest_id
        client.send_goal(
            goal,
            done_cb=self.guest_analysis_done
        )
        #client.wait_for_result()
        #result = client.get_result()
        #return result.description
    
    def guest_analysis_done(self, status, result) -> None:
        """Callback for the guest analysis"""
        rospy.loginfo(f"Guest analysis result: {result.description} for guest {result.guest_id}")
        self.guest_description[result.guest_id] = result.description
    
    def get_guest_description(self, guest_id: int) -> str:
        """Method to get the guest description stored"""
        if self.FAKE_TASK:
            return "sample_description"
        return self.guest_description[guest_id]

    def speak(self, text: str, now: bool = False) -> None:
        """Method to publish directly text to the speech node"""
        rospy.loginfo(f"[INFO] Speaking: {text}")
        
        if not self.FAKE_TASK:
            self.pub_speak.publish(text)
            #self.speak_client(text)
        
    
    def get_items_category(self, object_names) -> str:
        """Method to get the category of a list of object names"""
        if self.FAKE_TASK:
            self.fake_index += 1
            return "sample_category_" + str(self.fake_index)
        response = self.items_category_client(items=object_names)
        return response.category

    def get_object_category(self, object_name) -> str:
        """Method to get the category of a single object"""
        if self.FAKE_TASK:
            return random.choice(self.OBJECT_CATEGORIES)
        response = self.items_category_client(items=[object_name])
        return response.category
    
    def callbackKeyword(self, data):
        self.keyword_detected = True


if __name__ == "__main__":
    try:
        TasksHRI()
    except rospy.ROSInterruptException as e:
        rospy.logerr("Error: {}".format(e))
        pass