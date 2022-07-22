from ast import arg
import re
import sys
import rclpy
import argparse
from rclpy.node import Node
from rclpy.action import ActionClient

from toolhead_interfaces.action import ToolHeadAction
from toolhead_interfaces.msg import ToolHeadMode


class ToolHeadActionClient(Node):

    def __init__(self, argv=sys.argv):
        parser= argparse.ArgumentParser()
        parser.add_argument('-p' '--pusher', required=True,
            type=int, help='0 for push in | 1 for push out')
        parser.add_argument('-w' '--wheel', required=True,
            type=int, help='0 for spin in | 1 for spin out')
        super().__init__('toolhead_client_node')
        self.cli = ActionClient(self, ToolHeadAction, 'toolhead_action')
        self.args = parser.parse_args(argv[1:])


    def send_request(self):
        toolhead_goal = ToolHeadAction.Goal()
        if self.args.pusher == 0:
            toolhead_goal.pusher_action.operation = ToolHeadMode.PUSHER_IN
        elif self.args.pusher == 1:
            toolhead_goal.pusher_action.operation = ToolHeadMode.PUSHER_OUT
        else:
            self.get_logger().error('invalid number for pusher command!')
            return
        if self.args.wheel == 0:
            toolhead_goal.wheel_action.operation = ToolHeadMode.SPIN_IN
        elif self.args.wheel == 1:
            toolhead_goal.wheel_action.operation = ToolHeadMode.SPIN_OUT
        else:
            self.get_logger().error("invalid number for wheel command!")
            return
        self.cli.send_goal(toolhead_goal)
        self.get_logger().info("sent!")


def main(args=sys.argv):
    rclpy.init(args=args)
    action_client = ToolHeadActionClient(sys.argv)
    action_client.send_request()
    rclpy.spin(action_client)
    rclpy.shutdown()



if __name__ == '__main__':
    main()