#!/usr/bin/env python3

"""
Task manager for the Receptionist task of RoboCup @Home 2024
"""

# Import libraries
import rospy
import actionlib
import time
import copy
from statemachine import State, StateMachine

# ROS messages
from std_msgs.msg import String
from frida_vision_interfaces.msg import Person, PersonList
from frida_hri_interfaces.msg import Command, CommandList
from frida_hri_interfaces.msg import GuestAnalysisAction, GuestAnalysisFeedback, GuestAnalysisGoal, GuestAnalysisResult
from frida_hri_interfaces.srv import GuestInfo, GuestInfoResponse

# Python submodules
from hri_tasks import TasksHRI
from manipulation_tasks import TasksManipulation
from nav_tasks import TasksNav
from vision_tasks import TasksVision

SPEAK_TOPIC = "/speech/speak"
CONVERSATION_SERVER = "/conversation_as"
FACE_LOCATIONS_TOPIC = "/vision/person_list"

NAV_ENABLED = True
MANIPULATION_ENABLED = True
CONVERSATION_ENABLED = True
VISION_ENABLED = True

FAKE_NAV = False
FAKE_MANIPULATION = False
FAKE_HRI = False
FAKE_VISION = False

AREAS = ["nav", "manipulation", "hri", "vision"]


class Guest:
    """Class to store the information of the guest"""

    def __init__(self, guest_id: int = 0, name: str = "", favorite_drink: str = "", description: str = "") -> None:
        self.guest_id = guest_id
        self.name = name
        self.favorite_drink = favorite_drink
        self.description = description

    def set_info(self, name: str, favorite_drink: str) -> None:
        self.name = name
        self.favorite_drink = favorite_drink

    def set_description(self, description: str) -> None:
        self.description = description


class ReceptionistTaskManager(StateMachine):
    """Class to manage different tasks divided by categories"""

    waiting_guest = State(initial=True)
    self_introduction = State()
    request_guest_information = State()
    save_user_face = State()
    go_to_living_room = State()
    introduce_guest_1 = State()
    introduce_guest_2 = State()
    gaze_at_guest = State()
    find_free_seat = State()
    wait_user_to_sit = State()
    go_to_entrance = State()
    start = State()
    shutdown = State()

    start_task = start.to(waiting_guest)
    introduce_to_guest = waiting_guest.to(self_introduction)
    introduce_self = self_introduction.to(request_guest_information)
    request_info = request_guest_information.to(save_user_face)
    save_face = save_user_face.to(go_to_living_room)
    # go_living_room = go_to_living_room.to(
    #     introduce_guest_1) | go_to_living_room.to(introduce_guest_2)
    intro_guest_1 = go_to_living_room.to(introduce_guest_1)
    intro_guest_2 = go_to_living_room.to(introduce_guest_2)

    introduce_guest1 = introduce_guest_1.to(gaze_at_guest)
    introduce_guest2 = introduce_guest_2.to(gaze_at_guest)
    gaze_guest = gaze_at_guest.to(find_free_seat)
    find_seat = find_free_seat.to(wait_user_to_sit)
    wait_sit = wait_user_to_sit.to(
        go_to_entrance) | wait_user_to_sit.to(shutdown)

    go_entrance = go_to_entrance.to(waiting_guest)
    # handle_error = error.to(waiting_guest)
    shutdown_system = shutdown.to(shutdown)

    def __init__(self) -> None:
        self._node = rospy.init_node("receptionist_server")
        self._rate = rospy.Rate(200)

        self.subtask_manager = dict.fromkeys(AREAS, None)

        self.subtask_manager["hri"] = TasksHRI(fake=FAKE_HRI)

        self.subtask_manager["manipulation"] = TasksManipulation(
            fake=FAKE_MANIPULATION)
        self.subtask_manager["nav"] = TasksNav(fake=FAKE_NAV)

        self.subtask_manager["vision"] = TasksVision(fake=FAKE_VISION)
        self.detected_faces = []

        rospy.Subscriber(FACE_LOCATIONS_TOPIC, PersonList,
                         self.get_face_locations)

        self.current_past_state = None
        self.current_command = None
        self.current_queue = []
        self.perceived_information = ""

        self.following_face = True
        self.followed_person = "Unknown"
        self.arm_moving = False

        self.find_sit_attempts = 0

        self.current_guest = 1

        self.hostname = "Ale"
        self.host_drink = "water"

        self.guests = [
            Guest(0, self.hostname, self.host_drink, ""),
            Guest(1),
            Guest(2),
        ]

        self.host_identified = False
        rospy.loginfo("Initialized receptionist task manager")

        super().__init__()

    def get_face_locations(self, data: PersonList) -> None:
        """Callback to receive the face locations"""
        self.detected_faces = data.list

    def follow_face(self) -> bool:
        """Calls the arm joints server to follow a face
        Returns: Movement of the arm executed"""
        if FAKE_VISION:
            return True

        if self.following_face and not self.arm_moving and self.detected_faces:
            rospy.loginfo(f"Following {self.followed_person}")
            self.arm_moving = True
            for face in self.detected_faces:
                if face.name == self.followed_person:
                    self.subtask_manager["manipulation"].move_arm_joints(
                        face.x, face.y, clear_octomap=True)
                    self.arm_moving = False
                    self.detected_faces = []
                    return True
            self.arm_moving = False
        return False

    def on_enter_waiting_guest(self):
        rospy.loginfo("Waiting for guest")
        while not rospy.is_shutdown():
            self.followed_person = "Unknown"
            self.subtask_manager["manipulation"].move_arm_joints(
                0, 0, "face_detection", clear_octomap=True)
            if self.subtask_manager["vision"].check_person():
                self.follow_face()
                self.introduce_to_guest()

    def on_enter_self_introduction(self):
        rospy.loginfo("Self introduction")
        self.follow_face()
        self.subtask_manager["hri"].speak(
            "Hi, my name is Frida. I'll be your receptionist today. When my light blinks fast, I'm listening. Could you tell me your name and your favorite drink?", now=False)
        self.introduce_self()

    def on_enter_request_guest_information(self):
        rospy.loginfo("Request guest information")
        while not rospy.is_shutdown():
            name, drink = self.subtask_manager["hri"].get_guest_info(
                self.current_guest)
            if name != "error":
                self.guests[self.current_guest].set_info(name, drink)
                self.subtask_manager["hri"].speak(
                    f"Nice to meet you {name}, please stay in front of me while I recognize your face.", now=False)
                self.request_info()
            else:
                self.subtask_manager["hri"].speak(
                    "I'm sorry, I didn't get your information.", now=False)

    def on_enter_save_user_face(self):
        rospy.loginfo("Save user face")
        while not rospy.is_shutdown():
            if self.follow_face():
                self.subtask_manager["vision"].analyze_guest(
                    self.current_guest)
                self.subtask_manager["vision"].save_face_name(
                    self.guests[self.current_guest].name)
                self.subtask_manager["hri"].speak(
                    "I have saved your face, thank you. Please follow me to the living room.", now=False)
                self.save_face()
            else:
                self.subtask_manager["hri"].speak(
                    "I'm sorry, I couldn't recognize your face. Please stay in front of me.", now=False)

    def on_enter_go_to_living_room(self):
        rospy.loginfo("Go to living room")
        while not rospy.is_shutdown():
            self.subtask_manager["nav"].execute_command(
                "remember", "past location", "")
            self.subtask_manager["hri"].speak(
                "The host is already waiting for you there. Please stay behind me until I find your seat.", now=False)
            self.subtask_manager["manipulation"].move_arm_joints(
                0, 0, "face_detection", clear_octomap=True)
            self.subtask_manager["nav"].execute_command(
                "go", "living_room", "")
            if self.current_guest == 1:
                self.intro_guest_1()
            else:
                self.intro_guest_2()

    def on_enter_introduce_guest_1(self):
        rospy.loginfo("Introduce guest 1 to host")
        while not rospy.is_shutdown():
            description = self.subtask_manager["vision"].get_guest_description(
                1)
            self.guests[1].set_description(description)

            if not self.host_identified:
                self.followed_person = "Unknown"
                self.subtask_manager["manipulation"].move_arm_joints(
                    0, 0, "face_detection", clear_octomap=True)
                rospy.loginfo("Save host face")
                timeout_face = 0
                while not rospy.is_shutdown() and not self.host_identified and timeout_face < 10:
                    if not self.follow_face():
                        time.sleep(1)
                        timeout_face += 1
                        rospy.loginfo("Expecting unknown face")
                        continue

                    self.subtask_manager["vision"].analyze_guest(0)
                    self.subtask_manager["vision"].save_face_name(
                        self.guests[0].name)
                    description = self.subtask_manager["vision"].get_guest_description(
                        0)
                    self.guests[0].set_description(description)

                    self.subtask_manager["hri"].speak(
                        f"I have saved {self.guests[0].name}.", now=False)
                    self.host_identified = True

            self.subtask_manager["manipulation"].move_arm_joints(
                0, 0, "seat", clear_octomap=True)
            timeout_face = 0
            while not self.follow_face() and timeout_face < 10:
                time.sleep(1)
                timeout_face += 1
                rospy.loginfo("Expecting host face")

            self.subtask_manager["hri"].speak(
                f"Hi {self.guests[0].name}, this is {self.guests[1].name}. It's favorite drink is {self.guests[1].favorite_drink}", now=False)
            self.subtask_manager["hri"].speak(
                f"{self.guests[1].description}", now=False)
            self.introduce_guest1()

    def on_enter_introduce_guest_2(self):
        rospy.loginfo("Introduce guest 2 to host and guest 1")
        description = self.subtask_manager["vision"].get_guest_description(2)
        self.guests[2].set_description(description)

        introduced_to = 0
        face_locations = ["left_face", "face_detection", "right_face"]

        for location in face_locations:
            if introduced_to >= 2:
                break
            self.subtask_manager["manipulation"].move_arm_joints(
                0, 0, location, clear_octomap=True)
            self.detected_faces = []
            time.sleep(2)
            current_faces = copy.deepcopy(self.detected_faces)

            for face in current_faces:
                self.followed_person = face.name
                if self.follow_face():
                    introduced_to += 1
                    self.subtask_manager["hri"].speak(
                        f"Hi {face.name}, this is {self.guests[2].name}. It's favorite drink is {self.guests[2].favorite_drink}", now=False)
                    self.subtask_manager["hri"].speak(
                        f"{self.guests[2].description}", now=False)

        self.introduce_guest2()

    def on_enter_gaze_at_guest(self):
        rospy.loginfo("Gaze at guest")
        self.subtask_manager["manipulation"].move_arm_joints(
            0, 0, "back", clear_octomap=True)
        self.followed_person = self.guests[self.current_guest].name
        timeout_face = 0
        while not self.follow_face() and timeout_face < 10:
            time.sleep(1)
            timeout_face += 1
            rospy.loginfo("Expecting guest face")
        self.subtask_manager["hri"].speak(
            f"I'll find you a free seat {self.guests[self.current_guest].name}, please wait.", now=True)

        self.subtask_manager["manipulation"].move_arm_joints(
            0, 0, "seat", clear_octomap=True)
        self.gaze_guest()

    def on_enter_find_free_seat(self):
        rospy.loginfo("Find free seat")
        time.sleep(5)
        seat_angle = self.subtask_manager["vision"].find_seat()
        if seat_angle == 300:
            self.subtask_manager["hri"].speak(
                "I'm sorry, I couldn't find a free seat for you. Please sit where you prefer", now=True)
        else:
            self.subtask_manager["hri"].speak(
                "I have found a free seat for you, please follow the direction of my arm.", now=True)
            self.subtask_manager["manipulation"].move_arm_joints(
                seat_angle, 20, clear_octomap=True)
        self.find_seat()

    def on_enter_wait_user_to_sit(self):
        rospy.loginfo("Wait user to sit")
        time.sleep(2)
        timeout_face = 0
        while not self.follow_face() and timeout_face < 10:
            time.sleep(1)
            timeout_face += 1
            rospy.loginfo("Expecting guest face")

        self.subtask_manager["hri"].speak(
            "I've detected you took your seat. I'll go back to the entrance now.", now=True)
        if self.current_guest < 2:
            self.current_guest += 1
            self.wait_sit()
        else:
            self.subtask_manager["hri"].speak(
                "I have finished my task.", now=True)
            self.shutdown_system()

    def on_enter_go_to_entrance(self):
        rospy.loginfo("Go to entrance")
        self.subtask_manager["nav"].execute_command("go", "entrance", "")
        self.subtask_manager["manipulation"].move_arm_joints(
            0, 0, "face_detection", clear_octomap=True)
        self.go_entrance()

    def run(self) -> None:
        """Main loop for the task manager"""
        rospy.loginfo("Starting Receptionist task manager")
        self.subtask_manager["nav"].go_place("entrance")
        self.start_task()


if __name__ == "__main__":
    try:
        task_manager = ReceptionistTaskManager()
        task_manager.run()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
