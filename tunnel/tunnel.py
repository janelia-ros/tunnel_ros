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

from Phidget22.PhidgetException import *
from .latch import Latch, LatchInfo

import time

class TunnelInfo():
    def __init__(self):
        self.latches_info = {'right': LatchInfo(), 'left': LatchInfo()}

        self.latches_info['right'].stepper_joint_info.stepper_info.phidget_info.hub_port = 0
        self.latches_info['right'].stepper_joint_info.stepper_info.phidget_info.label = 'tunnel_0'
        self.latches_info['right'].stepper_joint_info.home_switch_info.phidget_info.hub_port = 1
        self.latches_info['right'].stepper_joint_info.home_switch_info.phidget_info.label = 'tunnel_0'
        self.latches_info['right'].stepper_joint_info.limit_switch_info.phidget_info.hub_port = 2
        self.latches_info['right'].stepper_joint_info.limit_switch_info.phidget_info.label = 'tunnel_0'
        self.latches_info['right'].stepper_joint_info.stepper_info.invert_direction = True

        self.latches_info['left'].stepper_joint_info.stepper_info.phidget_info.hub_port = 5
        self.latches_info['left'].stepper_joint_info.stepper_info.phidget_info.label = 'tunnel_0'
        self.latches_info['left'].stepper_joint_info.home_switch_info.phidget_info.hub_port = 4
        self.latches_info['left'].stepper_joint_info.home_switch_info.phidget_info.label = 'tunnel_0'
        self.latches_info['left'].stepper_joint_info.limit_switch_info.phidget_info.hub_port = 3
        self.latches_info['left'].stepper_joint_info.limit_switch_info.phidget_info.label = 'tunnel_0'
        self.latches_info['left'].stepper_joint_info.stepper_info.invert_direction = False


class Tunnel():
    def __init__(self, tunnel_info, name, logger):
        self.tunnel_info = tunnel_info
        self.name = name
        self.logger = logger

        self.latches = {}
        for name, info in self.tunnel_info.latches_info.items():
            self.latches[name] = Latch(info, self.name + '_' + name + "_latch", self.logger)

    def open(self):
        for name, latch in self.latches.items():
            latch.open()

    def close(self):
        for name, latch in self.latches.items():
            latch.close()

    def set_on_attach_handler(self, on_attach_handler):
        for name, latch in self.latches.items():
            latch.set_on_attach_handler(on_attach_handler)

    def _on_attach_handler(self, handle):
        for name, latch in self.latches.items():
            if latch.has_handle(handle):
                latch._on_attach_handler(handle)

    def is_attached(self):
        for name, latch in self.latches.items():
            if not latch.is_attached():
                return False
        return True

    def set_stepper_on_change_handlers(self, stepper_on_change_handler):
        for name, latch in self.latches.items():
            latch.stepper_joint.stepper.set_on_position_change_handler(stepper_on_change_handler)
            latch.stepper_joint.stepper.set_on_velocity_change_handler(stepper_on_change_handler)

    def set_stepper_on_change_handlers_to_disabled(self):
        for name, latch in self.latches.items():
            latch.stepper_joint.stepper.set_on_position_change_handler(None)
            latch.stepper_joint.stepper.set_on_velocity_change_handler(None)

    def set_stepper_on_stopped_handlers(self, stepper_on_stopped_handler):
        for name, latch in self.latches.items():
            latch.stepper_joint.stepper.set_on_stopped_handler(stepper_on_stopped_handler)

    def set_stepper_on_stopped_handlers_to_disabled(self):
        for name, latch in self.latches.items():
            latch.stepper_joint.stepper.set_on_stopped_handler(None)

    def set_limit_switch_handlers(self, limit_switch_handler):
        for name, latch in self.latches.items():
            latch.stepper_joint.set_limit_switch_handler(limit_switch_handler)

    def set_limit_switch_handlers_to_disabled(self):
        for name, latch in self.latches.items():
            latch.stepper_joint.set_limit_switch_handler_to_disabled()

    def home_latches(self):
        for name, latch in self.latches.items():
            latch.stepper_joint.home()

    def all_latches_homed(self):
        all_homed = True
        for name, latch in self.latches.items():
            if not latch.stepper_joint.homed:
                all_homed = False
                break
        return all_homed

    def any_latches_moving(self):
        any_moving = False
        for name, latch in self.latches.items():
            if latch.stepper_joint.stepper.is_moving():
                any_moving = True
                break
        return any_moving

    def latch_all(self):
        for name, latch in self.latches.items():
            latch.latch()

    def unlatch_all(self):
        for name, latch in self.latches.items():
            latch.unlatch()
