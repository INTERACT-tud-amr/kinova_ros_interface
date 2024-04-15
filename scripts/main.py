#!/usr/bin/env python3

import rospy
from threading import Thread
import time
import sys
from kinova_ros_interface.kinova_client import KinovaRobot
from kinova_ros_interface.utilities import DeviceConnection
from kinova_ros_interface.state import State

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64, Bool

PUBLISH_RATE = 40 #Hz


class ControlInterface():
    """A node that starts the control interface of the robot."""
    def __init__(self, mode="position") -> None:
        self.mode = mode

        self.pub_feedback = rospy.Publisher('/kinova/joint_states', JointState, queue_size=10)
        rospy.Subscriber("/kinova/command", Float64MultiArray, self.callback_command)
        rospy.Subscriber("/kinova/gripper", Float64, self.callback_gripper)
        rospy.Subscriber("/kinova/error_ack", Bool, self.callback_error_ack)

        
        self.state = State()
        self.start_threads()

        """Start the robot """
        with DeviceConnection.createTcpConnection() as router, DeviceConnection.createUdpConnection() as real_time_router:
            self.kinova = KinovaRobot(router=router, real_time_router=real_time_router, state=self.state)

            # for i in range(6):
            #     self.kinova.set_control_mode(i, "velocity")

            self.kinova.start_feedback()


    def start_threads(self):
            self.publishing_active = True
            publish_thread = Thread(target=self.start_publish_loop)
            publish_thread.start()

            spin_thread = Thread(target=self.start_spin_loop)
            spin_thread.start()

    
    def start_publish_loop(self):
        while self.publishing_active:
            self.publish_feedback()
            time.sleep(1/ PUBLISH_RATE)
    
    def start_spin_loop(self):
        rospy.spin()
        if rospy.is_shutdown():
            self.publishing_active = False
            self.kinova.stop_feedback()
            sys.exit()
    
    def publish_feedback(self):
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        # for i in range(self.kinova.actuator_count):
        for i in range(len(self.state.kinova_feedback.q)):
            name = "joint_" + str(i+1)
            js_msg.name.append(name)
            js_msg.position.append(self.state.kinova_feedback.q[i])
            js_msg.velocity.append(self.state.kinova_feedback.dq[i])
        self.pub_feedback.publish(js_msg)

    def stop_threads(self):
        self.publishing_active = False
        self.kinova.stop_feedback()
        

    def callback_command(self, msg):
        if self.mode == "position":
            self.state.kinova_command.q = msg.data
            self.kinova.set_high_level_position(self.state.kinova_command.q)
        if self.mode == "velocity":
            self.state.kinova_command.dq = msg.data
            self.kinova.set_high_level_velocity(self.state.kinova_command.dq)

    def callback_gripper(self, msg):
        self.kinova._move_gripper(msg.data)

    def callback_error_ack(self, msg):
        self.kinova.clear_faults()


if __name__ == "__main__":
    # Ros initialization
    rospy.init_node("kinova_driver")
    kinova_driver = ControlInterface("velocity")
    # rospy.spin()
