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
        if (goal_handle.request.pusher_action.operation == ToolHeadMode.PUSHER_OUT):
            pusher_action_string = "Pusher Out"
        elif (goal_handle.request.pusher_action.operation == ToolHeadMode.PUSHER_IN):
            pusher_action_string = "Pusher In"
        elif (goal_handle.request.pusher_action.operation == ToolHeadMode.PUSHER_MOVING):
            pusher_action_string = "Pusher Moving"
        elif (goal_handle.request.pusher_action.operation == ToolHeadMode.PUSHER_IDLE):
            pusher_action_string = "Pusher Idling"
        elif (goal_handle.request.pusher_action.operation == ToolHeadMode.ERROR):
            pusher_action_string = "What?! Why do send it to perform an error?!"


        if (goal_handle.request.wheel_action.operation == ToolHeadMode.SPIN_IN):
            wheel_action_string = "Spin In"
        elif (goal_handle.request.wheel_action.operation == ToolHeadMode.SPIN_OUT):
            wheel_action_string = "Spin Out"
        elif (goal_handle.request.wheel_action.operation == ToolHeadMode.SPIN_IDLE):
            wheel_action_string = "Spin Idling"
        elif (goal_handle.request.wheel_action.operation == ToolHeadMode.ERROR):
            wheel_action_string = "What?! Why do send it to perform an error?!"

        self.get_logger().info(f"Executing action: \
            Pusher Action:{pusher_action_string} \
            Spin Action: {wheel_action_string}")

        pusher_cmd = str(goal_handle.request.pusher_action.operation)
        wheel_cmd = str(goal_handle.request.wheel_action.operation)

        feedback = ToolHeadAction.Feedback()

        # in any case we where the pusher AND the spinning wheels need to be
        # called the wheel will need to spin first in these scenarios

        # wheel
        self.get_logger().info(f"attempting to set {wheel_action_string}")
        try:
            self.s.write(str.encode(str(wheel_cmd) + '\n'))
            status = self.s.readline().decode("utf-8","strict").rstrip()
            # feedback.wheel_state.operation =\
            #         goal_handle.request.wheel_action.operation
            if int(status) == goal_handle.request.wheel_action.operation:
                feedback.wheel_state.operation =\
                    goal_handle.request.wheel_action.operation
            else:
                feedback.wheel_state.operation = ToolHeadMode.ERROR

        except Exception as e:
            self.get_logger().info(f"error: {e}")
            self.get_logger().info(f"failed to serial write to \
                {wheel_action_string}")
            feedback.wheel_state.operation = ToolHeadMode.ERROR


        self.get_logger().info(f"attempting to set {pusher_action_string}")
        try:
            self.s.write(str.encode(str(pusher_cmd) + '\n'))
            status = self.s.readline().decode("utf-8","strict").rstrip()
            # feedback.pusher_state.operation =\
            #         goal_handle.request.pusher_action.operation
            if int(status) == goal_handle.request.pusher_action.operation:
                feedback.pusher_state.operation =\
                    goal_handle.request.pusher_action.operation
            else:
                feedback.pusher_state.operation = ToolHeadMode.ERROR
        except Exception as e:
            self.get_logger().info(f"error: {e}")
            self.get_logger().info(f"failed to serial write to \
                {pusher_action_string}")
            feedback.pusher_state.operation = ToolHeadMode.ERROR

        feedback.success = False
        # goal_handle.publish_feedback(feedback)

        success = False
        if(feedback.pusher_state.operation == goal_handle.request.pusher_action.operation\
            or feedback.wheel_state.operation == goal_handle.request.wheel_action):
            success = True
            self.get_logger().info("toolhead action succeeded")
            goal_handle.succeed()

        result = ToolHeadAction.Result()
        result.pusher_state.operation = feedback.pusher_state.operation
        result.wheel_state.operation = feedback.wheel_state.operation
        result.success = success
        self.get_logger().info("returning result")
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