import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from handy_msgs.action import Obstacle
from std_msgs.msg import Bool, Float32MultiArray, Float32, Int8
import time

print("RUNNING OBSTACLE SERVERRRRR")


class ObstacleDetector(Node):
    def __init__(self, args=None):
        super().__init__('OBSTACLE_SERVER')

        self._action_server = ActionServer(self, Obstacle, '/obstacle_detector', 
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def cancel_callback(self, goal):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        global pose
        global gps
        global points
        feedback_msg = Obstacle.Feedback()

        for e in range(10):
            self.get_logger().info('No obstacle detected')
            obstacle = Bool()
            obstacle.data = False
            feedback_msg.obstacle = obstacle
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        self.get_logger().info('Obstacle detected')
        obstacle = Bool()
        obstacle.data = True
        feedback_msg.obstacle = obstacle

        obs_type = Int8()
        obs_type.data = 1
        feedback_msg.type = obs_type

        obs_detected_before = Bool()
        obs_detected_before.data = False
        feedback_msg.detected_before = obs_detected_before

        obs_distance = Float32()
        obs_distance.data = 2.0
        feedback_msg.distance = obs_distance

        obs_has_size = Bool()
        obs_has_size.data = True
        feedback_msg.has_size = obs_has_size

        obs_size = Float32MultiArray()
        obs_size.data = [1.0, 1.0, 1.0]
        feedback_msg.size = obs_size

        obs_has_position = Bool()
        obs_has_position.data = True
        feedback_msg.has_position = obs_has_position
        
        obs_position = Float32MultiArray()
        obs_position.data = [1.0, 1.0, 1.0]
        feedback_msg.loc_position = obs_position
        goal_handle.publish_feedback(feedback_msg)

        result = Obstacle.Result()
        goal_handle.succeed()
        result = Obstacle.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    navfix = ObstacleDetector(args=args)
    rclpy.spin(navfix)
    rclpy.shutdown()

if __name__ == '__main__':
    main()