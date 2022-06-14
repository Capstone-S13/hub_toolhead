#!/usr/bin/env python3

from toolhead_service.srv import ToolHead
from toolhead_service.msg import ToolHeadAction

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
import serial

#from std_msgs.msg import String


class ToolheadService(Node):

    def __init__(self):
        super().__init__('toolhead_service_node')

        self.declare_param()
        self.toolhead_init()

        # Create Service
        self.srv = self.create_service(ToolHead, 'toolhead_interface', self.action_callback)

        self.s = serial.Serial(self.port, self.baud_rate)

    def declare_param(self):
        port          = '/dev/ttyUSB1'
        baud_rate         = 115200

        self.declare_parameter('port', port)
        self.declare_parameter('baud_rate', baud_rate)

    def toolhead_init(self):
        self.port          = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate     = self.get_parameter('baud_rate').get_parameter_value().integer_value
    
    def action_callback(self, request, response):

        self.get_logger().info('Incoming request: %s' % (request.toolhead_action))
        if (not response.toolhead_busy):
            if (request == "pusher_in"):
                # command = 0
                response.toolhead_busy = True
                self.s.write(str.encode("0"))
                self.s.readline()
                status = self.s.readline()
                if status.startswith('Done.'):
                    response.toolhead_busy = False
                response.toolhead_position = 0
            elif (request.pusher_action == ToolHeadAction.PUSHER_OUT\
                and request._action == ToolHeadAction.SPIN_IN):
                # command = 1
                response.toolhead_busy = True
                self.s.write(str.encode("1"))
                self.s.readline()
                status = self.s.readline()
                if status.startswith('Done.'):
                    response.s
                    response.toolhead_busy = False
                response.toolhead_position = 0
            

        return response

def main(args=None):
    rclpy.init(args=args)

    tool_head_service = ToolheadService()

    rclpy.spin(tool_head_service)

    tool_head_service.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()