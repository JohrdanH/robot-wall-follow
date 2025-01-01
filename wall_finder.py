# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node
# import the empty module from std_servs Service interface
from std_srvs.srv import Empty
# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the MyCustomServiceMessage module from custom_interfaces package
from custom_interfaces.srv import FindWall
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import time

class WallFinder(Node):

    def __init__(self):
        # Here you have the class constructor

        # call the class constructor to initialize the node as service_stop
        super().__init__('wall_finder')

        self.mutuallyexclusive_group_1 = MutuallyExclusiveCallbackGroup()
        self.mutuallyexclusive_group_2 = MutuallyExclusiveCallbackGroup()
        self.mutuallyexclusive_group_3 = MutuallyExclusiveCallbackGroup()
        # create the Service Server object
        # defines the type, name, and callback function
        self.srv = self.create_service(FindWall, 'find_wall', self.find_wall_callback, callback_group=self.mutuallyexclusive_group_1)
        # create the subscriber object
        # in this case, the subscriptor will be subscribed on /scan topic with a queue size of 10 messages.
        # use the LaserScan module for /scan topic
        # send the received info to the listener_callback method.
        self.laser_msg = LaserScan()
        self.flag = 0
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.mutuallyexclusive_group_2)  # is the most used to read LaserScan data and some sensor data.
        
        # create the Publisher object
        # in this case, the Publisher will publish on /cmd_vel topic with a queue size of 10 messages.
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)    

        # use the Twist module
        self.cmd = Twist()

        # define the timer period for 0.5 second
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.mutuallyexclusive_group_3)

        # Variables to track the process
        self.min_distance = 10
        self.offset = 0.05
        self.flag = 0
        self.waiting = 0
        self.done = False
        self.pause_w = 0.0
        self.i_dmin = 0
        
    def laser_callback(self, msg):
        self.laser_msg = msg
        # self.dist_to_front = self.laser_msg.ranges[0]
        # self.dist_to_side = self.laser_msg.ranges[540]
        # Values for real-world
        self.dist_to_side = self.laser_msg.ranges[180]
        self.dist_to_front = self.laser_msg.ranges[360]

    def timer_callback(self):
        if self.flag == 1:
            self.get_logger().info('Length: "%s" ' % str(len(self.laser_msg.ranges)))
            for i in range(0,len(self.laser_msg.ranges)):
                if self.min_distance > self.laser_msg.ranges[i]:
                    self.min_distance = self.laser_msg.ranges[i]
            self.get_logger().info('Min. Distance: "%s" m' % str(self.min_distance))
            if len(self.laser_msg.ranges) > 2 and self.min_distance < 5:
                self.flag = 2

        elif self.flag == 2:
            self.cmd.angular.z = 0.3
            self.get_logger().info('Min. Distance: "%s" m' % str(self.min_distance))
            self.get_logger().info('Distance to front : "%s" m' % str(self.dist_to_front))
            if self.min_distance + self.offset - 0.017 <= self.dist_to_front and self.dist_to_front <= self.min_distance + self.offset + 0.017:
                self.cmd.angular.z = 0.0
                self.flag = 3
        
        elif self.flag == 3:
            if self.dist_to_front < 0.3:
                self.cmd.linear.x = 0.0
                self.get_logger().info('Distance to front : "%s" m' % str(self.dist_to_front))
                self.flag = 4
            else:
                self.cmd.linear.x = 0.05

        elif self.flag == 4:
            self.get_logger().info('Distance to side wall : "%s" m' % str(self.dist_to_side))
            if 0.2 < self.dist_to_side and self.dist_to_side < 0.255:
                self.cmd.angular.z = 0.0
                self.done = True
                self.flag = 5

            else:
                self.cmd.angular.z = 0.25

        if  self.flag != 5:
            self.publisher_.publish(self.cmd)

    def find_wall_callback(self, request, response):
        self.flag = 1

        # Wait for the wall to be found (blocking loop)
        while not self.done:
            # Sleep for a short time to avoid busy waiting
            time.sleep(0.1)

        '''
        self.flag = 1
        j = 0
        # Wait for the wall to be found (blocking loop)
        while not self.done:
            if self.flag == 1:
                self.get_logger().info('Length: "%s" ' % str(len(self.laser_msg.ranges)))
                for i in range(0,len(self.laser_msg.ranges)):
                    if self.min_distance > self.laser_msg.ranges[i]:
                        self.min_distance = self.laser_msg.ranges[i]
                        self.i_dmin = i # i_dmin = 360 <---> 180°
                self.get_logger().info('Min. Distance: "%s" m' % str(self.min_distance))
                if len(self.laser_msg.ranges) > 2 and self.min_distance < 5:
                    self.flag = 2

            elif self.flag == 2:
                self.cmd.angular.z = 0.3 # self.pause is calculated based on this value
                self.pause = self.i_dmin*0.5*19.4/360.0 # 25.4 seg <---> 360°, 24.9 in simulation
                self.get_logger().info('i_dmin: "%s" m' % str(self.i_dmin))
                while (j <= self.pause):
                    self.publisher_.publish(self.cmd)
                    j += 0.1
                    time.sleep(0.1)
                j = 0
                self.flag = 3
            
            elif self.flag == 3:
                self.cmd.angular.z = 0.0
                self.get_logger().info('Pause F3: "%s" s' % str(self.pause))
                self.flag = 4

            elif self.flag == 4:
                
                self.dist_to_wall = self.min_distance - 0.3
                if self.dist_to_wall > 0:
                    self.cmd.linear.x = 0.1
                    self.pause_w = self.dist_to_wall/self.cmd.linear.x
                    self.flag = 5
                else:
                    self.flag = 6

            elif self.flag == 5:
                while (j <= self.pause_w):
                    self.publisher_.publish(self.cmd)
                    j += 0.1
                    time.sleep(0.1)
                j = 0
                self.cmd.linear.x = 0.0
                self.flag = 6
                
            elif self.flag == 6:
                self.cmd.angular.z = 0.3
                self.pause = 180*0.5*19.4/360.0
                self.flag = 7

            elif self.flag == 7:
                while (j <= self.pause):
                    self.publisher_.publish(self.cmd)
                    j += 0.1
                    time.sleep(0.1)
                j = 0
                self.cmd.angular.z = 0.0
                self.flag = 0
                self.done = True

            self.publisher_.publish(self.cmd)
            # time.sleep(0.1)
            '''
        response.wallfound = True
        self.get_logger().info("Wall found, sending response.")
        return response
                
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor  
    wall_finder = WallFinder()

    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(wall_finder)

    try:
        executor.spin()
    finally:
        wall_finder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()