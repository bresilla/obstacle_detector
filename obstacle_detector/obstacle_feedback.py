import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from handy_msgs.action import Obstacle
from std_msgs.msg import Bool, Float32MultiArray, Float32, Int8
import time


class ObstacleFeedback(Node):

    def __init__(self):
        super().__init__('OBSTACLE_FEEDBACK')
        self._action_client = ActionClient(self, Obstacle, '/obstacle_detector')

    def send_goal(self):
        goal_msg = Obstacle.Goal()
        goal_msg.start.data = True
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        detection = feedback.obstacle.data
        if detection == True:
            self.get_logger().info('Obstacle detected')
            self.get_logger().info('Obstacle type: {0}'.format(feedback.type.data))
            self.get_logger().info('Obstacle detected before: {0}'.format(feedback.detected_before.data))
            self.get_logger().info('Obstacle distance: {0}'.format(feedback.distance.data))
            self.get_logger().info('Obstacle has size: {0}'.format(feedback.has_size.data))
            self.get_logger().info('Obstacle size: {0}'.format(feedback.size.data))
            self.get_logger().info('Obstacle has position: {0}'.format(feedback.has_position.data))
            self.get_logger().info('Obstacle local position: {0}'.format(feedback.loc_position.data))
        else:
            self.get_logger().info('No obstacle detected')


def main(args=None):
    rclpy.init(args=args)
    action_client = ObstacleFeedback()
    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()