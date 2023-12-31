import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import rclpy
from rclpy.node import Node
from handy_msgs.action import Obstacle, Nav


class ObstacleFeedback(Node):
    def __init__(self):
        super().__init__('OBSTACLE_FEEDBACK')
        self._action_client = ActionClient(self, Obstacle, '/obstacle_detector')
        self._navclient = ActionClient(self, Nav, '/navigation')
        self.counter = 0


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

    async def feedback_callback(self, feedback_msg):
        self.counter += 1
        feedback = feedback_msg.feedback
        detection = feedback.obstacle.data
        if detection == True:
            goal_msg = Nav.Goal()
            goal_msg.abort.data = True

            self.get_logger().info('Obstacle detected')
            self.get_logger().info('Obstacle type: {0}'.format(feedback.type.data))
            self.get_logger().info('Obstacle detected before: {0}'.format(feedback.detected_before.data))
            self.get_logger().info('Obstacle distance: {0}'.format(feedback.distance.data))
            self.get_logger().info('Obstacle has size: {0}'.format(feedback.has_size.data))
            self.get_logger().info('Obstacle size: {0}'.format(feedback.size.data))
            self.get_logger().info('Obstacle has position: {0}'.format(feedback.has_position.data))
            # self.get_logger().info('Obstacle local position: {0}'.format(feedback.loc_position.data))
            self.get_logger().info('------------------------------------------------------------')
            
            self._navclient.wait_for_server()
            await self._navclient.send_goal_async(goal_msg)
        else:
            if self.counter % 1000 == 0:
                self.get_logger().info('No obstacle detected')


def main(args=None):
    rclpy.init(args=args)
    action_client = ObstacleFeedback()
    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()