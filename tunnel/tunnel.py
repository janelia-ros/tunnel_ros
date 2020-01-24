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
        self.latches_info['right'].stepper_joint_info.home_switch_info.phidget_info.hub_port = 1
        self.latches_info['right'].stepper_joint_info.limit_switch_info.phidget_info.hub_port = 2
        self.latches_info['right'].stepper_joint_info.stepper_info.invert_direction = True

        self.latches_info['left'].stepper_joint_info.stepper_info.phidget_info.hub_port = 5
        self.latches_info['left'].stepper_joint_info.home_switch_info.phidget_info.hub_port = 4
        self.latches_info['left'].stepper_joint_info.limit_switch_info.phidget_info.hub_port = 3
        self.latches_info['left'].stepper_joint_info.stepper_info.invert_direction = False


class Tunnel():
    def __init__(self, tunnel_info, name, logger):
        self.tunnel_info = tunnel_info
        self.name = name
        self.logger = logger

        self._setup_tunnel()

    def _setup_tunnel(self):
        self.latches = {}
        try:
            for name, info in self.tunnel_info.latches_info.items():
                self.latches[name] = Latch(info, self.name + '_' + name + "_latch", self.logger)

        except PhidgetException as e:
            self.logger.error(str(e))
            for name, latch in self.latches.items():
                latch.close()
            raise e

    # def calibrate_latches(self):
        # self.home_latches()
        # while not self.all_latches_homed():
        #     pass
        #     self.logger.info('waiting while any latches are homing...')
        #     time.sleep(0.2)
        # self.logger.info('all latches homed')

        # self.set_limit_switches_to_find_latch_positions()

        # while self.waiting_for_find_latch_positions_trigger:
        #     pass
        #     self.logger.info('waiting for find latch positions trigger...')
        #     time.sleep(0.2)
        # self.logger.info('find latch positions triggered')

        # while self.any_latches_moving():
        #     pass
        #     self.logger.info('waiting for latch positions to be found...')
        #     time.sleep(0.2)
        # self.logger.info('latch positions found')

        # self.unlatch_all()

        # while self.any_latches_moving():
        #     pass

    # def _trigger_find_latch_positions_handler(self, handle, state):
    #     for name, latch in self.latches.items():
    #         if not latch.stepper_joint.homed:
    #             return
    #         if not latch.stepper_joint.limit_switch.is_active():
    #             return
    #     limit_switch = latch.stepper_joint.limit_switch
    #     limit_switch.set_on_state_change_handler(self._find_latch_positions_handler)

    # def _find_latch_positions_handler(self, handle, state):
    #     for name, latch in self.latches.items():
    #         if not latch.stepper_joint.homed:
    #             return
    #         if latch.stepper_joint.limit_switch.is_active():
    #             return

    #     for name, latch in self.latches.items():
    #         latch.find_latch_position()
    #     self.waiting_for_find_latch_positions_trigger = False

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

    # def set_limit_switches_to_find_latch_positions(self):
    #     self.waiting_for_find_latch_positions_trigger = True
    #     for name, latch in self.latches.items():
    #         limit_switch = latch.stepper_joint.limit_switch
    #         limit_switch.set_on_state_change_handler(self._trigger_find_latch_positions_handler)
    #     msg = 'Waiting for headbar trigger to find latch positions...'
    #     self.logger.info(msg)

    # def set_limit_switches_to_latch(self):
    #     for name, latch in self.latches.items():
    #         limit_switch = latch.stepper_joint.limit_switch
    #         limit_switch.set_on_state_change_handler(self._latch_handler)

    def set_limit_switches_handler(self, limit_switches_handler):
        for name, latch in self.latches.items():
            limit_switch = latch.stepper_joint.limit_switch
            limit_switch.set_on_state_change_handler(limit_switches_handler)

    # def _latch_handler(self, handle, state):
    #     for name, latch in self.latches.items():
    #         if not latch.stepper_joint.homed:
    #             return
    #         if not latch.stepper_joint.limit_switch.is_active():
    #             return
    #     self.latch_all()

    def latch_all(self):
        for name, latch in self.latches.items():
            latch.latch()

    def unlatch_all(self):
        for name, latch in self.latches.items():
            latch.unlatch()
