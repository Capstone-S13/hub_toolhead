#!/usr/bin/env python3
from unittest import result
from requests import request
from rx import catch
from rcl_interfaces import msg
from toolhead_interfaces.msg import ToolHeadMode
from toolhead_interfaces.action import ToolHeadAction

import rclpy
from rclpy.action import ActionServer
from rclpy.parameter import Parameter
from rclpy.node import Node
import serial

#from std_msgs.msg import String


class ToolheadActionServer(Node):

    def __init__(self):
        super().__init__('toolhead_service_node')
        self.declare_param()
        self.toolhead_init()
        self.s = serial.Serial(self.port, self.baud_rate)
        self.action_server = ActionServer(
            self,
            ToolHeadAction,
            'toolhead_action',
            self.execute_callback
        )


    def execute_callback(self, goal_handle):
        pusher_action_string = ""
        wheel_action_string = ""
        # pusher action
        if (goal_handle.request.pusher_action == ToolHeadMode.PUSHER_OUT):
            pusher_action_string = "Pusher Out"
        elif (goal_handle.request.pusher_action == ToolHeadMode.PUSHER_IN):
            pusher_action_string = "Pusher In"

        if (goal_handle.request.wheel_action == ToolHeadMode.SPIN_IN):
            wheel_action_string = "Spin In"
        elif (goal_handle.request.wheel_action == ToolHeadMode.SPIN_OUT):
            wheel_action_string = "Spin Out"

        self.get_logger().info(f"Executing action: \
            Pusher Action:{pusher_action_string} \
            Spin Action: {wheel_action_string}")

        pusher_cmd = str(goal_handle.request.pusher_action)
        wheel_cmd = str(goal_handle.request.wheel_action)

        feedback = ToolHeadAction.Feedback()

        # in any case we where the pusher AND the spinning wheels need to be
        # called the wheel will need to spin first in these scenarios

        # wheel
        self.get_logger().info(f"attempting to set {wheel_action_string}")
        try:
            self.s.write(str.encode(str(wheel_cmd)))
            feedback.wheel_state = goal_handle.request.wheel_action

        except:
            self.get_logger().info(f"failed to serial write to \
                {wheel_action_string}")
            feedback.wheel_state = ToolHeadMode.ERROR


        self.get_logger().info(f"attempting to set {pusher_action_string}")
        try:
            self.s.write(str.encode(str(pusher_cmd)))
            feedback.pusher_state =\
                goal_handle.request.pusher_action
        except:
            self.get_logger().info(f"failed to serial write to \
                {pusher_action_string}")
            feedback.pusher_state = ToolHeadMode.ERROR

        goal_handle.publish_feedback(feedback)

        success = False
        if(feedback.pusher_state == goal_handle.request.pusher_action\
            or feedback.wheel_state == goal_handle.request.wheel_action):
            success = True
            goal_handle.succeed()

        result = ToolHeadAction.Result()
        result.pusher_state = feedback.pusher_state
        result.wheel_state = feedback.wheel_state
        result.sucess = success
        return result

    def declare_param(self):
        port          = '/dev/ttyUSB0'
        baud_rate         = 115200

        self.declare_parameter('port', port)
        self.declare_parameter('baud_rate', baud_rate)

    def toolhead_init(self):
        self.port          = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate     = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.get_logger().info('action server is up!')

def main(args=None):
    rclpy.init(args=args)

    tool_head_action_server = ToolheadActionServer()

    rclpy.spin(tool_head_action_server)

    tool_head_action_server.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()