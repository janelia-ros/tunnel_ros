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

from smart_cage_msgs.msg import TunnelState, TunnelControl

from .tunnel import Tunnel, TunnelInfo

import time
import threading
import datetime
import math

class TunnelNode(Node):
    def __init__(self):
        super().__init__('tunnel')

        self.tunnel_info = TunnelInfo()
        self.name = 'tunnel'
        self.logger = self.get_logger()

        self._attached_timer_period = 1
        self._attached_timer = None
        self._latched_timer_period = 5
        self._latched_timer = None

        self.latch_mode = True
        self.require_both_head_bar_sensors = True
        self.latched = False
        self.latch_disabled = False

        self.tunnel = Tunnel(self.name, self.logger, self.tunnel_info)
        self.tunnel.set_on_attach_handler(self._on_attach_handler)
        self.logger.info('opening tunnel phidgets...')
        self.tunnel.open()

        self._tunnel_state_publisher = self.create_publisher(TunnelState, 'tunnel_state')
        # self._tunnel_control_subscription = self.create_subscription(
        #     TunnelState,
        #     'tunnel_control',
        #     self._tunnel_control_callback,
        #     10)
        # self._tunnel_control_subscription  # prevent unused variable warning

    def _on_attach_handler(self, handle):
        self.tunnel._on_attach_handler(handle)
        if self._attached_timer is None:
            self._attached_timer = self.create_timer(self._attached_timer_period, self._attached_timer_callback)

    def _attached_timer_callback(self):
        self._attached_timer.cancel()
        self.destroy_timer(self._attached_timer)
        self._attached_timer = None
        if self.tunnel.is_attached():
            self.logger.info('tunnel is attached!')
            self.tunnel.set_stepper_on_change_handlers_to_disabled()
            self.tunnel.set_stepper_on_homed_handlers(self._homed_handler)
            self.tunnel.set_limit_switch_handlers(self._publish_tunnel_state_handler)
            self.tunnel.load_cell.set_on_voltage_ratio_change_handler(self._voltage_ratio_change_handler)
            self.tunnel.home_latches()

    def _publish_tunnel_state_handler(self, handle, value):
        if not self.tunnel.all_latches_homed:
            return
        tunnel_state = TunnelState()
        tunnel_state.datetime = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        now_frac, now_whole = math.modf(time.time())
        tunnel_state.nanosec = int(now_frac * 1e9)
        tunnel_state.load_cell_voltage_ratio = self.tunnel.load_cell.get_voltage_ratio()
        tunnel_state.right_head_bar_sensor_active = self.tunnel.latches['right'].stepper_joint.limit_switch.is_active()
        tunnel_state.left_head_bar_sensor_active = self.tunnel.latches['left'].stepper_joint.limit_switch.is_active()
        tunnel_state.right_latch_position = self.tunnel.latches['right'].stepper_joint.stepper.get_position()
        tunnel_state.left_latch_position = self.tunnel.latches['left'].stepper_joint.stepper.get_position()
        self._tunnel_state_publisher.publish(tunnel_state)

    def _voltage_ratio_change_handler(self, handle, value):
        self._publish_tunnel_state_handler(handle, value)

    def _homed_handler(self, handle):
        if not self.tunnel.all_latches_homed():
            return
        self.tunnel.set_stepper_on_change_handlers(self._publish_tunnel_state_handler)
        self.tunnel.set_stepper_on_stopped_handlers_to_disabled()
        self.tunnel.set_limit_switch_handlers(self._trigger_latch_handler)

    def _trigger_latch_handler(self, handle, state):
        self._publish_tunnel_state_handler(handle, state)
        if not self.tunnel.any_limit_switches_active():
            self.tunnel.deactivate_start_trial_trigger()
            return
        if self.require_both_head_bar_sensors and not self.tunnel.all_limit_switches_active():
            return
        self.tunnel.activate_start_trial_trigger()
        if self.latch_mode:
            self.latch_all()

    def _latched_handler(self, handle):
        if self.tunnel.any_latches_moving():
            return
        self.latched = True
        if self._latched_timer is None:
            self._latched_timer = threading.Timer(self._latched_timer_period, self._latched_timer_callback)
            self._latched_timer.start()
        self.logger.info('latched!')

    def _latched_timer_callback(self):
        self._latched_timer = None
        self.unlatch_all()

    def _unlatched_handler(self, handle):
        if self.tunnel.any_latches_moving():
            return
        self.latched = False
        self.latch_disabled = False

    def latch_all(self):
        if self.latch_disabled:
            return
        if not self.tunnel.all_steppers_in_step_control_mode():
            return
        self.tunnel.set_stepper_on_stopped_handlers(self._latched_handler)
        self.tunnel.latch_disabled = True
        self.tunnel.latch_all()

    def unlatch_all(self):
        self.tunnel.set_stepper_on_stopped_handlers(self._unlatched_handler)
        self.tunnel.unlatch_all()

    def shutdown(self):
        pass
        # self.unlatch_all()
        # self.tunnel.deactivate_start_trial_trigger()

    # def _tunnel_control_callback(self, msg):
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

    try:
        rclpy.spin(tunnel_node)
    except KeyboardInterrupt:
        pass

    tunnel_node.shutdown()
    # timeout_limit = 3
    # timeout_count = 0
    # while tunnel_node.latched and timeout_count < timeout_limit:
    #     time.sleep(1)
    #     timeout_count += 1
    tunnel_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
