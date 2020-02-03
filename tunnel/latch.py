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

from phidgets_python_api.stepper_joint import StepperJoint, StepperJointInfo
from phidgets_python_api.digital_input import DigitalInputInfo

class LatchInfo():
    def __init__(self):
        self.stepper_joint_info = StepperJointInfo()
        self.stepper_joint_info.stepper_info.acceleration = 100000
        self.stepper_joint_info.stepper_info.velocity_limit = 20000
        self.stepper_joint_info.stepper_info.current_limit = 0.5
        self.stepper_joint_info.stepper_info.holding_current_limit = 0.5
        self.stepper_joint_info.home_switch_info.active_low = False
        self.stepper_joint_info.limit_switch_info = DigitalInputInfo()
        self.stepper_joint_info.limit_switch_info.active_low = False
        self.find_latch_position_velocity_limit = 1000
        self.find_latch_position_target_position = 10000
        self.latch_position = 1300

class Latch():
    def __init__(self, latch_info, name, logger):
        self.latch_info = latch_info
        self.name = name
        self.logger = logger

        self.stepper_joint = StepperJoint(self.latch_info.stepper_joint_info, self.name, self.logger)

    def open(self):
        self.stepper_joint.open()

    def close(self):
        self.stepper_joint.close()

    def has_handle(self, handle):
        return self.stepper_joint.has_handle(handle)

    def set_on_attach_handler(self, on_attach_handler):
        self.stepper_joint.set_on_attach_handler(on_attach_handler)

    def _on_attach_handler(self, handle):
        self.stepper_joint._on_attach_handler(handle)

    def is_attached(self):
        return self.stepper_joint.is_attached()

    def latch(self):
        self.stepper_joint.stepper.set_target_position(self.latch_info.latch_position)

    def unlatch(self):
        self.stepper_joint.home()
