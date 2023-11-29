import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import message_filters
import threading
import numpy as np
from handy_msgs.action import Nav

print("RUNNING OBSTACLE SERVER")

pose = Pose()
gps = NavSatFix()
datum = NavSatFix()
points = None

class GetThePosition(Node):
    def __init__(self):
        super().__init__('get_the_position')
        self.path = Path()
        self.inited_waypoints = False
        self._odom_sub = message_filters.Subscriber(self, Odometry, '/fix/odom')
        self._gps_sub = message_filters.Subscriber(self, NavSatFix, '/fix/gps')
        self._datum_sub = message_filters.Subscriber(self, NavSatFix, '/fix/datum/gps')
        self.pose_sub = message_filters.ApproximateTimeSynchronizer(
            [self._odom_sub, self._gps_sub, self._datum_sub], 10, slop=10
        )
        self.pose_sub.registerCallback(self.pose_callback)
        self.path_pub = self.create_publisher(Path, "/fix/waypoints", 10)
        self.path_timer = self.create_timer(2.0, self.path_callback)

    def path_callback(self, msg=None):
        # self.get_logger().info('PUBLISHING PATH')
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = "map"
        if points is not None and self.inited_waypoints == False:
            for i in points:
                pose = PoseStamped()
                pose.header = self.path.header
                pose.pose.position.x = i.pose.position.x
                pose.pose.position.y = i.pose.position.y
                self.path.poses.append(pose)
            self.inited_waypoints = True
        self.path_pub.publish(self.path)
   
    def pose_callback(self, odom_msg, gps_msg, datum_msg):
        global pose
        global gps
        global datum
        pose.position = odom_msg.pose.pose.position
        pose.orientation = odom_msg.pose.pose.orientation
        gps = gps_msg
        datum = datum_msg


class GoToPosition(Node):
    def __init__(self):
        super().__init__('go_to_position')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ang_then_lin = True
        self.current_pose_ = Point()
        self.target_pose_ = Point()
        self.current_orientation_ = Quaternion()
        self._goal_handle = None
        self._goal_lock = threading.Lock()

        self._action_server = ActionServer(self, Nav, '/obstacle_detector', 
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup())

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def cancel_callback(self, goal):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    async def execute_callback(self, goal_handle):
        global pose
        global gps
        global points
        feedback_msg = Nav.Feedback()

        for e in range(100):
            self.get_logger().info('No obstacle detected')
            feedback_msg.obstacle = False
            goal_handle.publish_feedback(feedback_msg)

        self.get_logger().info('Obstacle detected')
        feedback_msg.obstacle = True
        feedback_msg.type = 1
        feedback_msg.detected_before = False
        feedback_msg.distance = 2.0
        feedback_msg.has_size = True
        feedback_msg.size = [1.0, 1.0, 1.0]
        feedback_msg.has_position = True
        feedback_msg.loc_position = [1.0, 1.0, 1.0]
        goal_handle.publish_feedback(feedback_msg)

        result = Nav.Result()
        goal_handle.succeed()
        result = Nav.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    try:
        getpos=GetThePosition()
        gotopos = GoToPosition()
        gotopos.log_file = False
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(getpos)
        executor.add_node(gotopos)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            getpos.destroy_node()
            gotopos.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':\
    main()