#!/usr/bin/env python3

"""
Task manager for the Storing task of RoboCup @Home 2024
"""

### Import libraries
import rospy
import actionlib
import copy
import math

### ROS messages
from std_msgs.msg import String, Bool
from frida_hri_interfaces.msg import Command, CommandList
from frida_hri_interfaces.msg import ConversateAction, ConversateFeedback, ConversateGoal, ConversateResult

### Python submodules
from hri_tasks import TasksHRI
from manipulation_tasks import TasksManipulation
from nav_tasks import TasksNav
from vision_tasks import TasksVision

NAV_ENABLED = True
MANIPULATION_ENABLED = True
CONVERSATION_ENABLED = True
VISION_ENABLED = True

FAKE_NAV = True
FAKE_MANIPULATION = True
FAKE_HRI = True
FAKE_VISION = True

ROOMS_LIST = ["entrance", "main_room", "kitchen", "bar", "trashbin", "dinig_room", "bathroom", "bedroom"]
FORBIDDEN_ROOM = "bedroom"
MAIN_ROOM = "main_room"

AREAS = ["nav", "manipulation", "hri", "vision"]
VISION_AVAILABLE_MODES = ["fast_execution", "robust"]

AREA_ENABLED = {
    "nav": NAV_ENABLED,
    "manipulation": MANIPULATION_ENABLED,
    "hri": CONVERSATION_ENABLED,
    "vision": VISION_ENABLED
}

class TaskManagerServer:
    """Class to manage different tasks divided by categories"""
    
    TASK_STATES = {
        "START": 0,
        "VISIT_ROOM": 1,
        "CHECK_RULES": 2,
        "SAY_RULE": 3, # Only activated once rule is broken
        "ASK_ACTION": 4,
        "GO_ENTRANCE": 5, # For no shoes rule
        "VERIFY_SHOES": 6,
        "GO_MAIN_ROOM": 7, # For forbidden room rule
        "GO_FORBIDDEN_ROOM": 8, # This returns to CHECK_RULES
        "GO_TRASHBIN": 9, # For trash rule
        "GO_KITCHEN-BAR": 10, # For drinks rule
        "ASK_DRINK": 11,
        "VERIFY_DRINK": 12,
        "VERIFY_TRASH": 13
    }
    
    RULES = {
        "NO_SHOES": 0,
        "FORBIDDEN_ROOM": 1,
        "TRASH": 2,
        "DRINKS": 3
    }
    
    RULE_ACTIONS = {
        0 : "Shoes are not allowed inside the house",
        1 : "Please do not enter this room, it is forbidden",
        2 : "Please help me pick that trash up",
        3 : "Please take a drink from the kitchen or the bar"
    }
    
    STATE_ENUM = {
        "IDLE": 0,
        "RECEIVE_COMMANDS": 1,
        "EXECUTING_COMMANDS": 2,
        "STOPPING": 3,
        "ERROR": 4,
        "SHUTDOWN": 5
    }

    COMMANDS_CATEGORY = {
        "nav" : ["go", "follow", "stop", "approach", "remember", "go_pose", "stop_follow"],
        "manipulation" : ["pick", "place", "grasp", "give", "open", "close", "pour", "observe"],
        "hri" : ["ask", "interact", "feedback", "analyze_objects"],
        "vision" : ["find", "identify", "count", "get_bag", "get_shelves"]
    }

    def __init__(self) -> None:
        self._node = rospy.init_node("task_manager_server")
        self._rate = rospy.Rate(200)
        # Creates an empty dictionary to store the subtask manager of each area
        self.subtask_manager = dict.fromkeys(AREAS, None)
        
        if MANIPULATION_ENABLED:
            self.subtask_manager["manipulation"] = TasksManipulation(fake=FAKE_MANIPULATION)
        if NAV_ENABLED:
            self.subtask_manager["nav"] = TasksNav(fake=FAKE_NAV)
        if VISION_ENABLED:
            self.subtask_manager["vision"] = TasksVision(fake=FAKE_VISION)
            self.vision_mode = "robust"
            if self.vision_mode not in VISION_AVAILABLE_MODES:
                rospy.logerr("[ERROR] Invalid vision mode")
                return
        if CONVERSATION_ENABLED:
            self.subtask_manager["hri"] = TasksHRI(fake=FAKE_HRI)
            self.subtask_manager["hri"].speak("Hi, my name is Frida. I will check that the party rules are being followed")   
        
        self.current_state = None
        self.current_command = None
        self.perceived_information = ""

        self.run()

    def execute_command(self, command: Command) -> int:
        """Method for executing a single command inside its area submodule"""

        rospy.loginfo(f"[INFO] Executing command: {command.action} -> {command.complement}")

        task_result = 0
        for area in AREAS:
            if command.action in TaskManagerServer.COMMANDS_CATEGORY[area] and AREA_ENABLED[area]:
                task_result = self.subtask_manager[area].execute_command(
                    command.action, command.complement, self.perceived_information
                )

        if task_result == -1 or task_result == 0:
            rospy.logerr("[ERROR] Error in task execution")
            return TaskManagerServer.STATE_ENUM["ERROR"]

        self.perceived_information += f"{command.action} {command.complement} {task_result} "
        return TaskManagerServer.STATE_ENUM["EXECUTING_COMMANDS"]

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        for area in AREAS:
            if self.current_command in TaskManagerServer.COMMANDS_CATEGORY[area]:
                self.subtask_manager[area].cancel_command()
    
    def filter_people_in_room(self, people: list, robot_room) -> list:
        people_inside = []
        for person in people:
            person_room = self.subtask_manager["nav"].get_point_room(person.Point3D)
            if person_room == robot_room:
                people_inside.append(person)
        return people_inside
    
    def get_closest_person(self, people: list) -> list:
        closest_person = None
        closest_distance = 1000
        for person in people:
            distance = math.sqrt(person.Point3D.point.x**2 + person.Point3D.point.y**2)
            if distance < closest_distance:
                closest_person = person
                closest_distance = distance
        return [closest_person]

    def run(self) -> None:
        """Main loop for the task manager"""
        self.current_state = TaskManagerServer.TASK_STATES["START"]
        while not rospy.is_shutdown():
            if self.current_state == TaskManagerServer.TASK_STATES["START"]:
                self.rooms_to_visit = copy.deepcopy(ROOMS_LIST)
                self.current_room = self.rooms_to_visit[0]
                self.current_state = TaskManagerServer.TASK_STATES["VISIT_ROOM"]
                
            elif self.current_state == TaskManagerServer.TASK_STATES["VISIT_ROOM"]:
                self.broken_rule = -1
                rospy.loginfo(f"[INFO] Going to {self.current_room}")
                result = self.execute_command(Command(action="go", complement=self.current_room))
                if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                    rospy.logerr("[ERROR] Error in navigation")
                self.current_state = TaskManagerServer.TASK_STATES["CHECK_RULES"]
            
            elif self.current_state == TaskManagerServer.TASK_STATES["CHECK_RULES"]:
                if self.subtask_manager["vision"].check_trash_rule():
                    self.broken_rule = TaskManagerServer.RULES["TRASH"]
                    self.current_state = TaskManagerServer.TASK_STATES["GO_TRASHBIN"]
                rospy.loginfo("[INFO] No trash rule was broken")
                people_observed = self.subtask_manager["vision"].get_people()
                rospy.loginfo("[INFO] Analyzing people")
                # filter people inside the room
                robot_room = self.subtask_manager["nav"].get_robot_current_room()
                
                people_inside = self.filter_people_in_room(people_observed, robot_room)
                
                for person in people_inside:
                    # TODO: Move robot arm so that it faces the closes person
                    # Check if the person is wearing shoes
                    if robot_room == FORBIDDEN_ROOM:
                        self.broken_rule = TaskManagerServer.RULES["FORBIDDEN_ROOM"]
                        self.current_state = TaskManagerServer.TASK_STATES["GO_MAIN_ROOM"]
                        rospy.loginfo("[INFO] No forbidden room rule was broken")
                    elif self.subtask_manager["vision"].check_has_shoes(person):
                        self.broken_rule = TaskManagerServer.RULES["NO_SHOES"]
                        rospy.loginfo("[INFO] No shoes rule was broken")
                        self.current_state = TaskManagerServer.TASK_STATES["GO_ENTRANCE"]
                    elif not self.subtask_manager["vision"].check_has_drink(person):
                        self.broken_rule = TaskManagerServer.RULES["DRINKS"]
                        self.current_state = TaskManagerServer.TASK_STATES["GO_KITCHEN-BAR"]
                        rospy.loginfo("[INFO] No drinks rule was broken")
            
                if self.broken_rule != -1:
                    self.subtask_manager["hri"].speak(TaskManagerServer.RULE_ACTIONS[self.broken_rule])
                else:
                    # Only pop once this room has no people breaking the rules, else return to check for more possible rule breakers
                    if len(self.rooms_to_visit) <= 0:
                        self.current_state = TaskManagerServer.TASK_STATES["START"]
                    else:
                        self.rooms_to_visit.pop(0)
                        self.current_room = self.rooms_to_visit[0]
                        self.current_state = TaskManagerServer.TASK_STATES["VISIT_ROOM"]
            
            ####### SHOES RULE #######
            elif self.current_state == TaskManagerServer.TASK_STATES["GO_ENTRANCE"]:
                self.subtask_manager["hri"].speak("Please accompany me to the entrance so you can take your shoes off")
                self.execute_command(Command(action="go", complement="entrance"))
                self.current_state = TaskManagerServer.TASK_STATES["VERIFY_SHOES"]
            
            elif self.current_state == TaskManagerServer.TASK_STATES["VERIFY_SHOES"]:
                rospy.sleep(5)
                people_observed = self.subtask_manager["vision"].get_people()
                closest_person = self.get_closest_person(people_observed)[0]
                # TODO: Move robot arm so that it faces the closes person
                if self.subtask_manager["vision"].check_has_shoes(closest_person):
                    self.subtask_manager["hri"].speak("Please take your shoes off")
                else:
                    self.subtask_manager["hri"].speak("Thank you for taking your shoes off")
                    self.current_state = TaskManagerServer.TASK_STATES["VISIT_ROOM"]
                    
            ####### FORBIDDEN ROOM RULE #######
            elif self.current_state == TaskManagerServer.TASK_STATES["GO_MAIN_ROOM"]:
                self.subtask_manager["hri"].speak("Please accompany me to the main room")
                self.execute_command(Command(action="go", complement=MAIN_ROOM))
                self.subtask_manager["hri"].speak("Please do not enter the forbidden room again")
                self.current_state = TaskManagerServer.TASK_STATES["VISIT_ROOM"]
            
            ####### TRASH RULE #######
            elif self.current_state == TaskManagerServer.TASK_STATES["GO_TRASHBIN"]:
                people_observed = self.subtask_manager["vision"].get_people()
                closest_person = self.get_closest_person(people_observed)[0]
                
                # TODO: Move robot arm so that it faces the closes person
                self.execute_command(Command(action="remember"))
                self.subtask_manager["hri"].speak("Please bring that trash to the trashbin, I will guide you there")
                self.execute_command(Command(action="go", complement="trashbin"))
                self.current_state = TaskManagerServer.TASK_STATES["VERIFY_TRASH"]
            
            elif self.current_state == TaskManagerServer.TASK_STATES["VERIFY_TRASH"]:
                if not self.subtask_manager["vision"].check_trash_rule():
                    self.subtask_manager["hri"].speak("Thank you for helping me with the trash, please throw it in the trashbin")
                    self.current_state = TaskManagerServer.TASK_STATES["VISIT_ROOM"]
                else:
                    self.execute_command(Command(action="go", complement="past location"))
                    self.current_state = TaskManagerServer.TASK_STATES["GO_TRASHBIN"]
                    
                
            ####### DRINKS RULE #######
            elif self.current_state == TaskManagerServer.TASK_STATES["GO_KITCHEN-BAR"]:
                self.subtask_manager["hri"].speak("Please accompany me to the kitchen or the bar to get a drink")
                self.execute_command(Command(action="go", complement="kitchen"))
                self.subtask_manager["hri"].speak("Please take a drink from the kitchen or the bar")
                self.current_state = TaskManagerServer.TASK_STATES["VERIFY_DRINK"]
            
            elif self.current_state == TaskManagerServer.TASK_STATES["VERIFY_DRINK"]:
                rospy.sleep(5)
                people_observed = self.subtask_manager["vision"].get_people()
                closest_person = self.get_closest_person(people_observed)[0]
                # TODO: Move robot arm so that it faces the closes person
                if self.subtask_manager["vision"].check_has_drink(closest_person):
                    self.subtask_manager["hri"].speak("Thank you for taking a drink")
                    self.current_state = TaskManagerServer.TASK_STATES["VISIT_ROOM"]
                else:
                    self.subtask_manager["hri"].speak("Please take a drink from the kitchen or the bar")
                
            self._rate.sleep()
        rospy.loginfo("[SUCCESS] Task Finished")

if __name__ == "__main__":
    try:
        TaskManagerServer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
