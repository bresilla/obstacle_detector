import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from handy_msgs.action import Obstacle
from std_msgs.msg import Bool, Float32MultiArray, Float32, Int8
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix
import message_filters
import time
import cv2
import numpy as np

def getBlobPositionInImage(rgb_image, color_name, min_size=1000):        
    color_hsv = {
        'blue': ([120, 100, 100], [150, 255, 255]),
        'green': ([35, 120, 50], [85, 255, 255]),
        'red': ([0, 120, 50], [10, 255, 255]),
        'yellow': ([25, 120, 50], [35, 255, 255]),
        'magenta': ([130, 120, 50], [150, 255, 255]),
        'cyan': ([75, 120, 50], [95, 255, 255]),
        'white': ([0, 0, 150], [180, 30, 255]),
        'black': ([0, 0, 0], [180, 255, 30]),
    }
    # Check if the specified color name is valid
    if color_name not in color_hsv:
        raise ValueError("Invalid color name")
    # Get the HSV range for the specified color
    lower_bound, upper_bound = color_hsv[color_name]
    # Convert image to HSV
    color = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
    # Get the mask with the specified color range
    mask = cv2.inRange(color, np.array(lower_bound), np.array(upper_bound))
    # Find contours in the masked image
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # If contours are found, find the center; otherwise, return x, y as None
    x = None
    y = None
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > min_size:
            moments = cv2.moments(largest_contour)
            x = int(moments["m10"] / moments["m00"])
            y = int(moments["m01"] / moments["m00"])
    return x, y, mask

def getPositionFromBlob(rgb_image, depth_image, color_name, negate=False):
    x, y, mask = getBlobPositionInImage(rgb_image, color_name)
    if x == None: return None, None, mask
    fov = 0.785
    width = 1280
    # Calculate the horizontal angle to the object
    angle = fov * (x - width / 2) / width
    # Calculate the distance to the object
    distance = depth_image[y, x]
    #if distans is inf, return None
    if distance == float("inf"): return None, None, mask
    return distance, -angle if negate else angle, mask


obstacle = Bool()
obs_type = Int8()
obs_detected_before = Bool()
obs_distance = Float32()
obs_has_size = Bool()
obs_size = Float32MultiArray()
obs_has_position = Bool()
loc_position = Pose()
geo_position = NavSatFix()
robot = Pose()

def obstacle_position(robot_x, robot_y, obstacle_distance, obstacle_angle):
    obstacle_x = robot_x + obstacle_distance * np.cos(np.radians(obstacle_angle))
    obstacle_y = robot_y + obstacle_distance * np.sin(np.radians(obstacle_angle))
    return obstacle_x, obstacle_y

class CameraSpinner(Node):
    def __init__(self, args=None):
        super().__init__('CAMERA_SPINNER')
        self.get_logger().info('CAMERA SPINNER')

        self.color_cam = message_filters.Subscriber(self, Image, '/Pioneer3at/camera/image_color')
        self.depth_cam = message_filters.Subscriber(self, Image, '/Pioneer3at/range_finder/image')
        self.odom_sub = message_filters.Subscriber(self, Pose, '/fix/odom')

        self.mask_pub = self.create_publisher(Image, '/mask', 10)
        
        self.img_sub = message_filters.ApproximateTimeSynchronizer(
            [
                self.color_cam, 
                self.depth_cam,
                self.odom_sub,
            ], 10, 0.1, allow_headerless=True)
        
        self.img_sub.registerCallback(self.img_callback)

    def img_callback(self, rgb_msg, depth_msg, odom_msg):
        global obstacle
        global obs_detected_before
        global obs_type
        global obs_distance
        global obs_has_size
        global obs_size
        global obs_has_position
        global loc_position
        global geo_position
        global robot

        bridge = CvBridge()
        rgb_image = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')
        depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passth rough')

        distance, angle, mask = getPositionFromBlob(rgb_image, depth_image, 'magenta')
        if distance == None or distance > 1.0:
            obstacle.data = False
        else:
            obstacle.data = True
            obs_type.data = 1
            obs_detected_before.data = False
            obs_distance.data = float(distance)
            obs_has_size.data = True
            obs_size.data = [0.0, 0.0, 0.0]
            obs_has_position.data = True
            robot.position = odom_msg.position
            robot.orientation = odom_msg.orientation
            obs_pose = obstacle_position(robot.position.x, robot.position.y, distance, angle)
            loc_position.position.x = obs_pose[0]
            loc_position.position.y = obs_pose[1]

        # self.get_logger().info('Obstacle detected')
        # self.get_logger().info('Distance: ' + str(distance))
        # self.get_logger().info('Angle: ' + str(angle))

        mask_msg = bridge.cv2_to_imgmsg(mask, encoding='passthrough')
        self.mask_pub.publish(mask_msg)

class ObstacleDetector(Node):
    def __init__(self, args=None):
        super().__init__('OBSTACLE_SERVER')
        self.get_logger().info('OBSTACLE SERVER INITIALIZED')
        self._action_server = ActionServer(self, Obstacle, '/obstacle_detector', 
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

    def cancel_callback(self, goal):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        global pose
        global gps
        global points
        feedback_msg = Obstacle.Feedback()
        global obstacle
        global obs_detected_before
        global obs_type
        global obs_distance
        global obs_has_size
        global obs_size
        global obs_has_position
        global loc_position
        global geo_position
        global robot
        # global loc_position
        while True:
            feedback_msg.obstacle = obstacle
            feedback_msg.type = obs_type
            feedback_msg.detected_before = obs_detected_before
            feedback_msg.distance = obs_distance
            feedback_msg.has_size = obs_has_size
            feedback_msg.size = obs_size
            feedback_msg.has_position = obs_has_position
            feedback_msg.loc_position = loc_position
            feedback_msg.geo_position = geo_position
            feedback_msg.robot = robot
            goal_handle.publish_feedback(feedback_msg)
            # self.get_logger().info('Feedback sent')

        result = Obstacle.Result()
        goal_handle.succeed()
        result = Obstacle.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    try:
        obs_d = ObstacleDetector()
        cam_s = CameraSpinner()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(obs_d)
        executor.add_node(cam_s)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            obs_d.destroy_node()
            cam_s.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':\
    main()