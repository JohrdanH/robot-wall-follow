import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_interfaces.action import OdomRecord

# import the Odometry module from geometry_msgs interface
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist
import time
class OdomRecorderServer(Node):

    def __init__(self):
        super().__init__('odom_recorder_server')

        self.mutuallyexclusive_group_1 = MutuallyExclusiveCallbackGroup()
        self.mutuallyexclusive_group_2 = MutuallyExclusiveCallbackGroup()

        self._action_server = ActionServer(self, OdomRecord, 'record_odom',self.execute_callback,callback_group=self.mutuallyexclusive_group_1) 

        self.first_odom = Point()
        self.last_odom = Point()
        self.subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.mutuallyexclusive_group_2)  # is the most used to read LaserScan data and some sensor data.
        
        self.last_x = 0.0
        self.last_y = 0.0
        self.curr_distance = 0.0
        self.total_distance = 0.0
        self.dist_fl = 0.0
        self.odom_record = []

    def odom_callback(self, msg):
        self.last_odom.x = msg.pose.pose.position.x
        self.last_odom.y = msg.pose.pose.position.y

    def execute_callback(self, goal_handle):
        
        self.get_logger().info('Executing goal...')

        feedback_msg = OdomRecord.Feedback()
        
        self.first_odom.x = self.last_odom.x
        self.first_odom.y = self.last_odom.y
        self.get_logger().info('First Odom reading in x: "%s" ' % str(self.first_odom.x))
        self.get_logger().info('First Odom reading in y: "%s" ' % str(self.first_odom.y))
        time.sleep(0.5)

        while True:
            self.last_x = self.last_odom.x
            self.last_y = self.last_odom.y
            self.odom_record.append(self.last_odom)
            time.sleep(1)

            self.get_logger().info('Last Odom reading in x: "%s" ' % str(self.last_odom.x))
            self.get_logger().info('Last Odom reading in y: "%s" ' % str(self.last_odom.y))
            
            self.curr_distance = (((self.last_odom.x - self.last_x) ** 2) + ((self.last_odom.y - self.last_y) ** 2) ) ** 0.5
            self.total_distance = self.total_distance + self.curr_distance
            feedback_msg.current_total = self.total_distance
            self.get_logger().info('Total distance: {0} '.format(feedback_msg.current_total))

            goal_handle.publish_feedback(feedback_msg)
            self.dist_fl = (((self.first_odom.x - self.last_odom.x) ** 2) + ((self.first_odom.y - self.last_odom.y) ** 2) ) ** 0.5
            self.get_logger().info('Distance btw. first and last odom: "%s" ' % str(self.dist_fl))

            if self.dist_fl < 0.08:
                break

        goal_handle.succeed()
        result = OdomRecord.Result()
        result.list_of_odoms = self.odom_record
        self.get_logger().info('Result: {0}'.format(result.list_of_odoms))
        return result

def main(args=None):
    rclpy.init(args=args)

    odom_recorder_server = OdomRecorderServer()

    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(odom_recorder_server)

    try:
        executor.spin()
    finally:
        odom_recorder_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()