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

from smart_cage_msgs.msg import TunnelState

from pathlib import Path
import datetime
import csv

class TunnelDataWriterNode(Node):
    def __init__(self):
        super().__init__('tunnel_data_writer')

        self.logger = self.get_logger()

        self._tunnel_state_subscription = self.create_subscription(
            TunnelState,
            'tunnel_state',
            self._tunnel_state_callback,
            10)
        self._tunnel_state_subscription  # prevent unused variable warning

        self.path = Path.home() / 'smart_cage_data' / datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        self.path.mkdir(parents=True)
        self.logger.info('created directory: ' + str(self.path))
        self.data_path = self.path / 'data.txt'
        self.data_file = open(self.data_path, 'w', newline='')
        self.fieldnames = ['time_in_seconds',
                           'load_cell_voltage_ratio',
                           'right_head_bar_sensor_state',
                           'left_head_bar_sensor_state',
                           'latch_position',
                           'now']
        self.data_writer = csv.DictWriter(self.data_file,
                                          delimiter=' ',
                                          quotechar='|',
                                          quoting=csv.QUOTE_MINIMAL,
                                          fieldnames=self.fieldnames)
        self.data_writer.writeheader()

    def _tunnel_state_callback(self, msg):
        msg_dict = {field: getattr(msg, field) for field in msg._fields_and_field_types if hasattr(msg, field)}
        self.data_writer.writerow(msg_dict)

    def close_files(self):
        self.data_file.close()

def main(args=None):
    rclpy.init(args=args)

    tunnel_data_writer_node = TunnelDataWriterNode()

    rclpy.spin(tunnel_data_writer_node)

    tunnel_data_writer_node.close_files()
    tunnel_data_writer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
