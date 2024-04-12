from kortex_api.RouterClient import RouterClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, ActuatorConfig_pb2

from utilities import DeviceConnection
from state import State
from rate_counter import RateCounter
from specifications import Position, actuator_ids, ranges

import numpy as np
from threading import Thread, Event
import time
from typing import Literal


class KinovaRobot():
    def __init__(self, 
                 base: BaseClient = None,
                 base_cyclic: BaseCyclicClient = None,
                 router: RouterClient = None,
                 real_time_router: RouterClient = None,
                 actuator_config: ActuatorConfigClient = None,) -> None:
        
        if None in [base, base_cyclic, actuator_config]:
            self.base = BaseClient(router)
            self.base_cyclic = BaseCyclicClient(real_time_router)
            self.actuator_config = ActuatorConfigClient(router)
        else:
            self.base = base
            self.base_cyclic = base_cyclic
            self.actuator_config = actuator_config

        self.actuator_count = self.base.GetActuatorCount().count
        self.state = State()
        
        # High-level control "HLC" (40Hz) or Low-level control "LLC" (1000Hz)
        self.mode = "HLC"
        # Set Servoing Mode
        self._set_servoing_mode(Base_pb2.SINGLE_LEVEL_SERVOING) # A second client is attempting to control the robot while it is in single-level servoing mode

        # Refresh
        self.refresh_feedback()

        # Initialize commands
        self._initialize_command()

        self.rate_counter = RateCounter(1000)


        # print(Base_pb2.ServoingMode.Name(self.servoing_mode))
        # print(self.actuator_config.GetControlMode(1).control_mode)
        # print(self.get_control_modes())
    def clear_faults(self) -> None:
        """Clear the faults."""
        self.base.ClearFaults()

    def _set_servoing_mode(self, value: int) -> None:
        """Set the servoing mode of the robot."""
        self.changing_servoing_mode = True
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = value
        self.base.SetServoingMode(base_servo_mode)
        self._update_modes()    #default is position mode
        self.changing_servoing_mode = False

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

    def _update_modes(self) -> None:
        """Update the modes."""
        self.servoing_mode = self.base.GetServoingMode().servoing_mode
        actuator_modes = []
        for n in range(self.actuator_count):
            _id = actuator_ids[n]
            actuator_modes.append(self.actuator_config.GetControlMode(_id).control_mode)
        self.actuator_modes = actuator_modes
    
    def _initialize_command(self) -> None:
        self.command = BaseCyclic_pb2.Command()
        for n in range(self.actuator_count):
            actuator_command = BaseCyclic_pb2.ActuatorCommand()
            actuator_command.flags = 1
            actuator_command.position = self.feedback.actuators[n].position
            actuator_command.velocity = self.feedback.actuators[n].velocity
            self.command.actuators.extend([actuator_command])

    def get_control_modes(self):
        """Get the control mode of an actuator."""
        return [
            ActuatorConfig_pb2.ControlMode.Name(self.actuator_modes[n])
            for n in range(self.actuator_count)
        ]
    
    def get_position(self, joint: int) -> float:
        position = getattr(self.feedback.actuators[joint], "position")
        lower_bound = ranges["position"][joint][0]
        upper_bound = ranges["position"][joint][1]
        position -= 360 if position > upper_bound else 0

        return np.deg2rad(position)
    

    def get_velocity(self, joint: int) -> float:
        velocity = getattr(self.feedback.actuators[joint], "velocity")
        return np.deg2rad(velocity)
    

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
        print("Closing connection with arm...")
        self.active = False

    def _refresh_loop(self) -> None:
        while self.active:
            self.refresh_feedback()
            self.update_state()
            self.rate_counter.count()
    

    def home(self) -> bool:
        """Move the arm to the home position."""
        self._high_level_move(Position.home)


    def zero(self) -> bool:
        """Move the arm to the zero position."""
        self._high_level_move(Position.zero)

    def set_joint_speed(self, velocity):
        self._high_level_vel(velocity)

    def _high_level_vel(self, velocity) -> None:
        joint_speeds_cmd = Base_pb2.JointSpeeds()
        for n, vel in enumerate(velocity):
            joint_speed = joint_speeds_cmd.joint_speeds.add()
            joint_speed.joint_identifier = n
            joint_speed.value = vel

        self.base.SendJointSpeedsCommand(joint_speeds_cmd)
        time.sleep(2)
        self.base.Stop()

        # action = Base_pb2.Action()
        # action.name = "vel"
        # action.application_data = ""
        # for n, vel in enumerate(velocity):
        #     joint_speed = action.send_joint_speeds.joint_speeds.add()
        #     joint_speed.joint_identifier = n
        #     joint_speed.value = vel

        # event = Event()
        # notification_handle = self.base.OnNotificationActionTopic(
        #     self._check_for_end_or_abort(event), Base_pb2.NotificationOptions()
        # )
        # self.base.ExecuteAction(action)
        # time.time(2)
        # self.base.Unsubscribe(notification_handle)

        # self.base.Stop()


    def _high_level_move(self, position: Position) -> None:
        """Perform a high level move."""
        action = Base_pb2.Action()
        action.name = position.name
        action.application_data = ""

        for n, pos in enumerate(position.position):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = n
            joint_angle.value = pos

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

    def set_velocity_mode(self):
        #TODO: copy feedback to command
        for n in range(self.actuator_count):
            self.set_control_mode(n, "velocity")
        self._set_servoing_mode(Base_pb2.SINGLE_LEVEL_SERVOING)



  

if __name__=="__main__":
    """Start the robot """
    with DeviceConnection.createTcpConnection() as router, DeviceConnection.createUdpConnection() as real_time_router:
        kinova = KinovaRobot(router=router, real_time_router=real_time_router)
        # kinova.clear_faults()
        kinova.start_feedback_in_new_thread()
        kinova.zero()

        vel = [0, 0, 0, 0, 50, 0]
        # kinova.set_velocity_mode()
        # kinova.set_joint_speed(vel)

        kinova.stop_feedback()
        


        
