from toolhead_interfaces.action import ToolHeadAction
from toolhead_interfaces.msg import ToolHeadMode
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


class ToolHeadActionClient(Node):

    def __init__(self):
        super().__init__('toolhead_client_node')
        self.cli = ActionClient(self, ToolHeadAction, 'toolhead_action')


    def send_request(self):
        toolhead_goal = ToolHeadAction.Goal()
        toolhead_goal.pusher_action.operation = ToolHeadMode.PUSHER_IN
        toolhead_goal.wheel_action.operation = ToolHeadMode.SPIN_IN
        self.cli.send_goal(toolhead_goal)
        self.get_logger().info("sent!")


def main(args=None):
    rclpy.init(args=args)

    action_client = ToolHeadActionClient()
    action_client.send_request()
    rclpy.spin(action_client)



if __name__ == '__main__':
    main()