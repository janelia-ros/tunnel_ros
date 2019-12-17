# Copyright (c) 2019, Howard Hughes Medical Institute
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from Phidget22.Phidget import *
from Phidget22.PhidgetException import *
from Phidget22.Devices.Stepper import *
from Phidget22.Devices.DigitalInput import *

from .PhidgetHelperFunctions import *

from time import time
import math

class Joint:
    ACCELERATION = 10000
    VELOCITY_LIMIT = 10000
    HOME_VELOCITY_LIMIT = 1000
    HOME_TARGET_POSITION = -10000
    CURRENT_LIMIT = 0.140
    HOLDING_CURRENT_LIMIT = 0
    ATTACHMENT_TIMEOUT = 5000

    def __init__(self, stepper_channel_info, home_switch_channel_info, name):
        self.name = name
        try:
            self._stepper = Stepper()
            self._home_switch = DigitalInput()
        except PhidgetException as e:
            DisplayError(e)
            raise
        self._logger = None
        self._setup_channel(self._stepper, stepper_channel_info)
        self._setup_channel(self._home_switch, home_switch_channel_info)

    def set_logger(self, logger):
        self._logger = logger

    def set_publish_joint_state(self, publish_joint_state):
        self._publish_joint_state = publish_joint_state
        self._stepper.setOnPositionChangeHandler(publish_joint_state)
        self._stepper.setOnVelocityChangeHandler(publish_joint_state)

    def _on_attach_handler(self, ph):
        try:
            channel_class_name = ph.getChannelClassName()
            serial_number = ph.getDeviceSerialNumber()
            channel = ph.getChannel()
            hub_port = ph.getHubPort()
            if ph.getIsHubPortDevice():
                msg = 'home switch {0} attached on hub port {1} on serial number {2}'.format(self.name, hub_port, serial_number)
                self._logger.info(msg)
            else:
                msg = 'stepper {0} attached on hub port {1} on serial number {2}'.format(self.name, hub_port, serial_number)
                self._logger.info(msg)

            try:
                ph.setDataInterval(100)
            except AttributeError:
                pass
            except PhidgetException as e:
                DisplayError(e)
                return

        except PhidgetException as e:
            DisplayError(e)
            traceback.print_exc()
            return

    def _on_detach_handler(self, ph):
        try:
            channel_class_name = ph.getChannelClassName()
            serial_number = ph.getDeviceSerialNumber()
            channel = ph.getChannel()
            if ph.getIsHubPortDevice():
                msg = 'home switch {0} detached on hub port {1} on serial number {2}'.format(self.name, hub_port, serial_number)
                self._logger.info(msg)
            else:
                msg = 'stepper {0} detached on hub port {1} on serial number {2}'.format(self.name, hub_port, serial_number)
                self._logger.info(msg)

        except PhidgetException as e:
            DisplayError(e)
            traceback.print_exc()
            return

    def _on_error_handler(self, ph, error_code, error_string):
        self._logger.error('[Phidget Error Event] -> ' + error_string + ' (' + str(error_code) + ')\n')

    def _setup_channel(self,channel,info):
        channel.setDeviceSerialNumber(info.deviceSerialNumber)
        channel.setHubPort(info.hubPort)
        channel.setIsHubPortDevice(info.isHubPortDevice)
        channel.setChannel(info.channel)

        channel.setOnAttachHandler(self._on_attach_handler)
        channel.setOnDetachHandler(self._on_detach_handler)
        channel.setOnErrorHandler(self._on_error_handler)

    def open_wait_for_attachment(self):
        self._stepper.openWaitForAttachment(self.ATTACHMENT_TIMEOUT)
        self._home_switch.openWaitForAttachment(self.ATTACHMENT_TIMEOUT)
        self._setup()

    def _setup(self):
        self._stepper.setAcceleration(self.ACCELERATION)
        self._stepper.setCurrentLimit(self.CURRENT_LIMIT)
        self._stepper.setVelocityLimit(self.VELOCITY_LIMIT)
        self._stepper.setHoldingCurrentLimit(self.HOLDING_CURRENT_LIMIT)
        self.enable()

    def home(self):
        if self._home_switch.getState():
            self._stepper.setVelocityLimit(self.HOME_VELOCITY_LIMIT)
            self._stepper.setTargetPosition(self.HOME_TARGET_POSITION)
            while self._home_switch.getState():
                pass
            self._stepper.setVelocityLimit(0.0)
            self._stepper.addPositionOffset(-self._stepper.getPosition())
            self._stepper.setTargetPosition(0.0)
            self._publish_joint_state(None,None)
            self._stepper.setVelocityLimit(self.VELOCITY_LIMIT)
        self._logger.info('{0} homed'.format(self.name))

    def close(self):
        self._stepper.setOnPositionChangeHandler(None)
        self._stepper.close()
        self._home_switch.close()

    def get_position(self):
        return self._stepper.getPosition()

    def get_velocity(self):
        return self._stepper.getVelocity()

    def enable(self):
        self._stepper.setEngaged(True)

    def disable(self):
        self._stepper.setEngaged(False)

    def set_target_position(self, target_position):
        self._stepper.setTargetPosition(target_position)

    def set_velocity_limit(self, velocity_limit):
        self._stepper.setVelocityLimit(velocity_limit)

class Tunnel(Node):
    X_JOINT_STEPPER_HUB_PORT = 0
    Y_JOINT_STEPPER_HUB_PORT = 1
    Z_JOINT_STEPPER_HUB_PORT = 2
    X_JOINT_HOME_SWITCH_HUB_PORT = 5
    Y_JOINT_HOME_SWITCH_HUB_PORT = 4
    Z_JOINT_HOME_SWITCH_HUB_PORT = 3

    def __init__(self):
        super().__init__('tunnel')
        self._joint_state_publisher = self.create_publisher(JointState, 'tunnel_joint_state', 10)
        self._joint_target_subscription = self.create_subscription(
            JointState,
            'tunnel_joint_target',
            self.joint_target_callback,
            10)
        self._joint_target_subscription  # prevent unused variable warning
        self._joints = {}
        self._setup_joints()

    def _setup_joints(self):
        try:
            stepper_channel_info = ChannelInfo()
            stepper_channel_info.deviceSerialNumber = Phidget.ANY_SERIAL_NUMBER
            stepper_channel_info.isHubPortDevice = False
            stepper_channel_info.channel = 0
            stepper_channel_info.isVint = True
            stepper_channel_info.netInfo.isRemote = False

            home_switch_channel_info = ChannelInfo()
            home_switch_channel_info.deviceSerialNumber = Phidget.ANY_SERIAL_NUMBER
            home_switch_channel_info.isHubPortDevice = True
            home_switch_channel_info.channel = 0
            home_switch_channel_info.isVINT = True
            home_switch_channel_info.netInfo.isRemote = False

            stepper_channel_info.hubPort = self.X_JOINT_STEPPER_HUB_PORT
            home_switch_channel_info.hubPort = self.X_JOINT_HOME_SWITCH_HUB_PORT
            name = 'x'
            self._joints[name] = Joint(stepper_channel_info, home_switch_channel_info, name)
            self._joints[name].set_logger(self.get_logger())
            self._joints[name].set_publish_joint_state(self._publish_joint_state)

            stepper_channel_info.hubPort = self.Y_JOINT_STEPPER_HUB_PORT
            home_switch_channel_info.hubPort = self.Y_JOINT_HOME_SWITCH_HUB_PORT
            name = 'y'
            self._joints[name] = Joint(stepper_channel_info, home_switch_channel_info, name)
            self._joints[name].set_logger(self.get_logger())
            self._joints[name].set_publish_joint_state(self._publish_joint_state)

            stepper_channel_info.hubPort = self.Z_JOINT_STEPPER_HUB_PORT
            home_switch_channel_info.hubPort = self.Z_JOINT_HOME_SWITCH_HUB_PORT
            name = 'z'
            self._joints[name] = Joint(stepper_channel_info, home_switch_channel_info, name)
            self._joints[name].set_logger(self.get_logger())
            self._joints[name].set_publish_joint_state(self._publish_joint_state)

            try:
                self._joints['x'].open_wait_for_attachment()
                self._joints['y'].open_wait_for_attachment()
                self._joints['z'].open_wait_for_attachment()
            except PhidgetException as e:
                raise EndProgramSignal('Program Terminated: Open Failed')

        except PhidgetException as e:
            DisplayError(e)
            traceback.print_exc()
            for name, joint in self._joints.items():
                joint.close()
            return 1
        except EndProgramSignal as e:
            self.get_logger().info(e)
            for name, joint in self._joints.items():
                joint.close()
            return 1

        def home_all():
            for name, joint in self._joints.items():
                joint.home()

        home_all()

    def disable_all_joints(self):
        for name, joint in self._joints.items():
            joint.disable()


    def _publish_joint_state(self, ch, position):
        joint_state = JointState()
        joint_state.header = Header()
        now_frac, now_whole = math.modf(time())
        joint_state.header.stamp.sec = int(now_whole)
        joint_state.header.stamp.nanosec = int(now_frac * 1e9)
        for name, joint in self._joints.items():
            joint_state.name.append(name)
            joint_state.position.append(joint.get_position())
            joint_state.velocity.append(joint.get_velocity())
        self._joint_state_publisher.publish(joint_state)

    def joint_target_callback(self, msg):
        if len(msg.name) == len(msg.velocity) == len(msg.position):
            targets = zip(msg.name, msg.velocity, msg.position)
            for name, velocity, position in targets:
                try:
                    self._joints[name].set_velocity_limit(velocity)
                    self._joints[name].set_target_position(position)
                except KeyError:
                    pass
        elif len(msg.name) == len(msg.position):
            targets = zip(msg.name, msg.position)
            for name, position in targets:
                try:
                    self._joints[name].set_target_position(position)
                except KeyError:
                    pass

def main(args=None):
    rclpy.init(args=args)

    tunnel = Tunnel()

    rclpy.spin(tunnel)

    tunnel.disable_all_joints()
    tunnel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
