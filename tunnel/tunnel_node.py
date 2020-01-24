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

from .tunnel import Tunnel, TunnelInfo

from time import time
import math

class TunnelNode(Node):
    def __init__(self):
        super().__init__('tunnel')

        self.tunnel_info = TunnelInfo()
        self.name = 'tunnel'
        self.logger = self.get_logger()
        self.tunnel = Tunnel(self.tunnel_info, self.name, self.logger)

        self._setup_tunnel_node()

    def _setup_tunnel_node(self):
        pass
        # self._joint_state_publisher = self.create_publisher(JointState, 'tunnel_joint_state', 10)
        # self._joint_target_subscription = self.create_subscription(
        #     JointState,
        #     'tunnel_joint_target',
        #     self._joint_target_callback,
        #     10)
        # self._joint_target_subscription  # prevent unused variable warning

        # for name, latch in self.tunnel.latches.items():
        #     latch.stepper_joint.stepper.set_on_position_change_handler(self._publish_joint_state)
        #     latch.stepper_joint.stepper.set_on_velocity_change_handler(self._publish_joint_state)

    # def latch(self):
    #     for name, joint in self.joints.items():
    #         joint.stepper.set_target_position(self.tunnel_info.latch_position)

    # def _find_latch_positions(self):
    #     for name, latch_switch in self.latch_switches.items():
    #         if not latch_switch.is_active():
    #             latch_switch.set_on_state_change_handler(self._find_latch_position_handler)
    #             stepper = self.joints[name].stepper
    #             stepper.set_velocity_limit(1000)
    #             stepper.set_target_position(10000)

    # def _find_latch_position_handler(self, handle, state):
    #     for name, latch_switch in self.latch_switches.items():
    #         if latch_switch.is_handle(handle):
    #             if latch_switch.is_active():
    #                 latch_position = self.joints[name].stepper.get_position()
    #                 msg = '{0 latch position is {1}'.format(latch_switch.name, latch_position)
    #                 self.logger.info(msg)
    #                 latch_switch.set_on_state_change_handler(None)

    # def disable_all_joints(self):
    #     for name, joint in self.joints.items():
    #         joint.stepper.disable()

    # def _publish_joint_state(self, handle, value):
    #     joint_state = JointState()
    #     joint_state.header = Header()
    #     now_frac, now_whole = math.modf(time())
    #     joint_state.header.stamp.sec = int(now_whole)
    #     joint_state.header.stamp.nanosec = int(now_frac * 1e9)
    #     for name, joint in self.joints.items():
    #         joint_state.name.append(name)
    #         joint_state.position.append(joint.stepper.get_position())
    #         joint_state.velocity.append(joint.stepper.get_velocity())
    #     self._joint_state_publisher.publish(joint_state)

    # def _joint_target_callback(self, msg):
    #     if len(msg.name) == len(msg.velocity) == len(msg.position):
    #         targets = zip(msg.name, msg.velocity, msg.position)
    #         for name, velocity, position in targets:
    #             try:
    #                 self.joints[name].stepper.set_velocity_limit(velocity)
    #                 self.joints[name].stepper.set_target_position(position)
    #             except KeyError:
    #                 pass
    #     elif len(msg.name) == len(msg.position):
    #         targets = zip(msg.name, msg.position)
    #         for name, position in targets:
    #             try:
    #                 self.joints[name].stepper.set_target_position(position)
    #             except KeyError:
    #                 pass


def main(args=None):
    rclpy.init(args=args)

    tunnel_node = TunnelNode()

    rclpy.spin(tunnel_node)

    tunnel_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
