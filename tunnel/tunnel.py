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

import Phidget22.Devices.VoltageRatioInput
from phidgets_python_api.phidget import PhidgetComposite
from phidgets_python_api.voltage_ratio_input import VoltageRatioInput, VoltageRatioInputInfo
from phidgets_python_api.digital_output import DigitalOutput, DigitalOutputInfo

from .latch import Latch, LatchInfo

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

        self.load_cell_info = VoltageRatioInputInfo()
        self.load_cell_info.phidget_info.hub_port = 0
        self.load_cell_info.phidget_info.label = 'tunnel_1'
        self.load_cell_info.bridge_gain = Phidget22.Devices.VoltageRatioInput.BridgeGain.BRIDGE_GAIN_64
        self.load_cell_info.voltage_ratio_change_trigger = 0.0

        self.start_trial_trigger_info = DigitalOutputInfo()
        self.start_trial_trigger_info.phidget_info.hub_port = 4
        self.start_trial_trigger_info.phidget_info.label = 'tunnel_1'
        self.start_trial_trigger_activated_duty_cycle = 0.5
        self.start_trial_trigger_deactivated_duty_cycle = 0.0


class Tunnel(PhidgetComposite):
    def __init__(self, name, logger, tunnel_info):
        super().__init__(name, logger)
        self.tunnel_info = tunnel_info

        self.latches = {}
        for name, info in self.tunnel_info.latches_info.items():
            latch_name = self.name + '_' + name + "_latch"
            latch = Latch(latch_name, self.logger, info)
            self.add(latch)
            self.latches[name] = latch

        load_cell_name = self.name + '_load_cell'
        self.load_cell = VoltageRatioInput(load_cell_name,
                                           self.logger,
                                           self.tunnel_info.load_cell_info)
        self.add(self.load_cell)

        start_trial_trigger_name = self.name + '_start_trial_trigger'
        self.start_trial_trigger = DigitalOutput(start_trial_trigger_name,
                                                 self.logger,
                                                 self.tunnel_info.start_trial_trigger_info)
        self.add(self.start_trial_trigger)

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

    def set_stepper_on_homed_handlers(self, on_homed_handler):
        for name, latch in self.latches.items():
            latch.stepper_joint.set_on_homed_handler(on_homed_handler)

    def set_limit_switch_handlers(self, limit_switch_handler):
        for name, latch in self.latches.items():
            latch.stepper_joint.set_limit_switch_handler(limit_switch_handler)

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

    def all_limit_switches_active(self):
        all_active = True
        for name, latch in self.latches.items():
            if not latch.stepper_joint.limit_switch.is_active():
                all_active = False
                break
        return all_active

    def any_limit_switches_active(self):
        any_active = False
        for name, latch in self.latches.items():
            if latch.stepper_joint.limit_switch.is_active():
                any_active = True
                break
        return any_active

    def all_steppers_in_step_control_mode(self):
        all_in_step_control_mode = True
        for name, latch in self.latches.items():
            if not latch.stepper_joint.stepper.in_step_control_mode():
                all_in_step_control_mode = False
                break
        return all_in_step_control_mode

    def latch_all(self):
        for name, latch in self.latches.items():
            latch.latch()

    def unlatch_all(self):
        for name, latch in self.latches.items():
            latch.unlatch()

    def activate_start_trial_trigger(self):
        duty_cycle = self.tunnel_info.start_trial_trigger_activated_duty_cycle
        self.start_trial_trigger.set_duty_cycle(duty_cycle)

    def deactivate_start_trial_trigger(self):
        duty_cycle = self.tunnel_info.start_trial_trigger_deactivated_duty_cycle
        self.start_trial_trigger.set_duty_cycle(duty_cycle)
