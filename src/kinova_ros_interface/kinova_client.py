from kortex_api.RouterClient import RouterClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, ActuatorConfig_pb2


import numpy as np
from threading import Thread, Event
import time
from typing import Literal

from .specifications import Position, actuator_ids, ranges
from .state import State


class KinovaRobot():
    def __init__(self, 
                 base: BaseClient = None,
                 base_cyclic: BaseCyclicClient = None,
                 router: RouterClient = None,
                 real_time_router: RouterClient = None,
                 actuator_config: ActuatorConfigClient = None,
                 state: State = None) -> None:
        
        if None in [base, base_cyclic, actuator_config]:
            self.base = BaseClient(router)
            self.base_cyclic = BaseCyclicClient(real_time_router)
            self.actuator_config = ActuatorConfigClient(router)
        else:
            self.base = base
            self.base_cyclic = base_cyclic
            self.actuator_config = actuator_config

        self.actuator_count = self.base.GetActuatorCount().count
        self.state = state
        
        # High-level control "HLC" (40Hz) or Low-level control "LLC" (1000Hz)
        self.mode = "HLC"
        # Set Servoing Mode
        self._set_servoing_mode(Base_pb2.SINGLE_LEVEL_SERVOING) # A second client is attempting to control the robot while it is in single-level servoing mode

        # Refresh
        self.refresh_feedback()

        # Initialize commands
        # self._initialize_command()

        # self.rate_counter = RateCounter(1000)

    def clear_faults(self) -> None:
        """Clear the faults."""
        self.base.ClearFaults()

    def get_position(self, joint: int) -> float:
        position = getattr(self.feedback.actuators[joint], "position")
        lower_bound = ranges["position"][joint][0]
        upper_bound = ranges["position"][joint][1]
        position -= 360 if position > upper_bound else 0

        return np.deg2rad(position)
    

    def get_velocity(self, joint: int) -> float:
        velocity = getattr(self.feedback.actuators[joint], "velocity")
        return np.deg2rad(velocity)

    def get_gripper_position(self) -> float:
        """Get the gripper position."""
        return (self.feedback.interconnect.gripper_feedback.motor[0].position) / 100
    

    def update_state(self) -> None:
        for n in range(self.actuator_count):
            self.state.kinova_feedback.q[n] = self.get_position(n)
            self.state.kinova_feedback.dq[n] = self.get_velocity(n)
            self.state.kinova_feedback.fault[n] = self.feedback.actuators[n].fault_bank_a

    def refresh_feedback(self) -> None:
        self.feedback =  self.base_cyclic.RefreshFeedback()

    def start_feedback_in_new_thread(self) -> None:
        thread = Thread(target=self.start_feedback)
        thread.start()

    def start_feedback(self) -> None:
        self.active = True
        self._refresh_loop()

    def stop_feedback(self, *args: any) -> None:
        """Stop the update loop."""
        print("Closing feedback loop with arm...")
        self.active = False

    def _refresh_loop(self) -> None:
        while self.active:
            self.refresh_feedback()
            self.update_state()
    

    def _set_servoing_mode(self, value: int) -> None:
        """Set the servoing mode of the robot."""
        self.changing_servoing_mode = True
        base_servo_mode = Base_pb2.ServoingModeInformation()
        print("servo mode: ", base_servo_mode)
        base_servo_mode.servoing_mode = value
        self.base.SetServoingMode(base_servo_mode)
        self._update_modes()    #default is position mode
        self.changing_servoing_mode = False

    def _update_modes(self) -> None:
        """Update the modes."""
        self.servoing_mode = self.base.GetServoingMode().servoing_mode
        actuator_modes = []
        for n in range(self.actuator_count):
            _id = actuator_ids[n]
            actuator_modes.append(self.actuator_config.GetControlMode(_id).control_mode)
        self.actuator_modes = actuator_modes

    def set_control_mode(
        self, joint: int, mode: Literal["position", "velocity", "current"]
    ) -> None:
            """Set the control mode of an actuator."""
            mode = getattr(ActuatorConfig_pb2, mode.upper())
            control_mode_information = ActuatorConfig_pb2.ControlModeInformation()
            control_mode_information.control_mode = mode
            _id = actuator_ids[joint]
            self.actuator_config.SetControlMode(control_mode_information, _id)
            self._update_modes()

    def _initialize_command(self) -> None:
        self.command = BaseCyclic_pb2.Command()
        for n in range(self.actuator_count):
            actuator_command = BaseCyclic_pb2.ActuatorCommand()
            actuator_command.flags = 1
            actuator_command.position = self.feedback.actuators[n].position
            actuator_command.velocity = self.feedback.actuators[n].velocity
            self.command.actuators.extend([actuator_command])

    def _move_gripper(self, pos_msg) -> None:
        gripper_command = Base_pb2.GripperCommand()
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger = gripper_command.gripper.finger.add()
        finger.finger_identifier = 1
        pos = self.get_gripper_position() + pos_msg
        print(pos)
        if pos >= 0.0 and pos <= 0.8:
            finger.value = pos
            self.base.SendGripperCommand(gripper_command)
        else:
            print("Invalid command for gripper")

    # def copy_feedback_to_command(self, joint: int = None) -> None:
    #     """Copy the feedback to the command message."""
    #     for prop in ["position", "velocity", "current_motor"]:
    #         for n in range(self.actuator_count) if joint is None else [joint]:
    #             value = getattr(self.feedback.actuators[n], prop)
    #             setattr(self.command.actuators[n], prop, value)

    def set_high_level_position(self, position) -> None:
        """Perform a high level move."""
        action = Base_pb2.Action()
        action.name = ""
        action.application_data = ""

        for n, pos in enumerate(position):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = n
            joint_angle.value = pos

        return self._execute_action(action)

    def set_high_level_velocity(self, velocity) -> None:
        # joint_speeds = Base_pb2.JointSpeeds()


        # for n, speed in enumerate(velocity):
        #     joint_speed = joint_speeds.joint_speeds.add()
        #     joint_speed.joint_identifier = n
        #     joint_speed.value = speed
        #     joint_speed.duration = 0
        # self.base.SendJointSpeedsCommand(joint_speeds)
        # # self.base.Stop()
        print(self.base.GetServoingMode())
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        # self.base.SetServoingMode(base_servo_mode)
        print(base_servo_mode)
        joint_speeds = Base_pb2.JointSpeeds()
        SPEED = 20.0


        print ("Sending the joint speeds for 10 seconds...")
        for times in range(4):
            del joint_speeds.joint_speeds[:]
            if times % 2:
                speeds = [-SPEED, 0.0, 0.0, SPEED, 0.0, 0.0]
            else:
                speeds = [SPEED, 0.0, 0.0, -SPEED, 0.0, 0.0]
            i = 0
            for speed in speeds:
                joint_speed = joint_speeds.joint_speeds.add()
                joint_speed.joint_identifier = i 
                joint_speed.value = speed
                joint_speed.duration = 0
                i = i + 1
            
            self.base.SendJointSpeedsCommand(joint_speeds)
            time.sleep(2.5)

        print ("Stopping the robot")
        self.base.Stop()


    def _execute_action(self, action: Base_pb2.Action = None) -> bool:
        event = Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self._check_for_end_or_abort(event), Base_pb2.NotificationOptions()
        )

        self.base.ExecuteAction(action)
        finished = event.wait(30)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print(f"Position {action.name} reached")
        else:
            print("Timeout on action notification wait")
        return finished

    def _check_for_end_or_abort(self, event: Event) -> callable:
        """Return a closure checking for END or ABORT notifications."""

        def check(notif: Base_pb2.ActionNotification, event: Event = event) -> None:
            if notif.action_event in [Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT]:
                event.set()

        return check
