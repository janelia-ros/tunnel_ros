# Copyright (c) 2020, Howard Hughes Medical Institute
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

from phidgets_python_api.stepper_joint import StepperJoint, StepperJointInfo

from time import time
import math

class Tunnel(Node):
    def __init__(self):
        super().__init__('tunnel')
        self._joint_state_publisher = self.create_publisher(JointState, 'tunnel_joint_state', 10)
        self._joint_target_subscription = self.create_subscription(
            JointState,
            'tunnel_joint_target',
            self._joint_target_callback,
            10)
        self._joint_target_subscription  # prevent unused variable warning

        self._joints_info = {'right': StepperJointInfo(), 'left': StepperJointInfo()}
        right = self._joints_info['right']
        right.stepper_info.channel_info.hub_port = 0
        right.home_switch_info.channel_info.hub_port = 1
        right.stepper_info.acceleration = 100000
        right.stepper_info.velocity_limit = 20000
        right.stepper_info.current_limit = 0.3
        right.stepper_info.holding_current_limit = 0.5
        right.stepper.invert_direction = True
        left = self._joints_info['left']
        left.stepper_info.channel_info.hub_port = 5
        left.home_switch_info.channel_info.hub_port = 4
        left.stepper_info.acceleration = 100000
        left.stepper_info.velocity_limit = 20000
        left.stepper_info.current_limit = 0.3
        left.stepper_info.holding_current_limit = 0.5
        left.stepper.invert_direction = False

        self._joints = {}
        self._setup_joints()

    def _setup_joints(self):
        try:
            try:
                for name, info in self.self._joints_info.items():
                    self._joints[name] = Joint(info)
            except:
                raise EndProgramSignal('Program Terminated: Open Failed')

        except PhidgetException as e:
            DisplayError(e)
            traceback.print_exc()
            for name, joint in self._joints.items():
                joint.close()
            return 1
        except EndProgramSignal as e:
            self.get_logger().info(str(e))
            for name, joint in self._joints.items():
                joint.close()
            raise EndProgramSignal(str(e))

        for name, joint in self._joints.items():
            joint.home()

    def _publish_joint_state(self, ch, position):
        joint_state = JointState()
        joint_state.header = Header()
        now_frac, now_whole = math.modf(time())
        joint_state.header.stamp.sec = int(now_whole)
        joint_state.header.stamp.nanosec = int(now_frac * 1e9)
        for name, joint in self._joints.items():
            joint_state.name.append(name)
            joint_state.position.append(joint.stepper.get_position())
            joint_state.velocity.append(joint.stepper.get_velocity())
        self._joint_state_publisher.publish(joint_state)

    def _joint_target_callback(self, msg):
        if len(msg.name) == len(msg.velocity) == len(msg.position):
            targets = zip(msg.name, msg.velocity, msg.position)
            for name, velocity, position in targets:
                try:
                    self._joints[name].stepper.set_velocity_limit(velocity)
                    self._joints[name].stepper.set_target_position(position)
                except KeyError:
                    pass
        elif len(msg.name) == len(msg.position):
            targets = zip(msg.name, msg.position)
            for name, position in targets:
                try:
                    self._joints[name].stepper.set_target_position(position)
                except KeyError:
                    pass

    def disable_all_joints(self):
        for name, joint in self._joints.items():
            joint.stepper.disable()


def main(args=None):
    rclpy.init(args=args)

    tunnel = Tunnel()

    rclpy.spin(tunnel)

    tunnel.disable_all_joints()
    tunnel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
