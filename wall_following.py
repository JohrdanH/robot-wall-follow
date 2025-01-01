import rclpy
# import the ROS2 python libraries
from rclpy.node import Node

import sys
from threading import Thread
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
# import the FindWall module from custom_interfaces.srv interface
from custom_interfaces.srv import FindWall
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile

from rclpy.action import ActionClient
from custom_interfaces.action import OdomRecord

class WallFollower(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('wall_follower')

        # create the Service Client object
        # defines the name and type of the Service Server you will work with.
        self.client = self.create_client(FindWall, 'find_wall')
        # checks once per second if a Service matching the type and name of the client is available.
        while not self.client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')

        # create an FindWall request
        self.req = FindWall.Request()

        self._action_client = ActionClient(self, OdomRecord, 'record_odom')

        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        # in this case, the subscriptor will be subscribed on /scan topic with a queue size of 10 messages.
        # use the LaserScan module for /scan topic
        # send the received info to the listener_callback method.
        self.laser_msg = LaserScan()
        self.dist_to_wall = 0.0
        self.front_laser = 0.0
        self.flag = 0
        self.resp = False
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  # is the most used to read LaserScan data and some sensor data.
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def send_request(self):
       return self.client.call(self.req)

    def send_goal(self):
        goal_msg = OdomRecord.Goal()

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

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
        self.get_logger().info('Result: {0}'.format(result.list_of_odoms))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Received feedback: {0}'.format(feedback.current_total))

    def laser_callback(self, msg):
        self.laser_msg = msg
        # Values for simulation
        # self.dist_to_wall = self.laser_msg.ranges[540] 
        # self.front_laser = self.laser_msg.ranges[0]
        # Values for real-world
        self.dist_to_wall = self.laser_msg.ranges[180]
        self.front_laser = self.laser_msg.ranges[360]

    def timer_callback(self):
        if self.resp == True:
            # print the data
            self.get_logger().info('Distance to wall: "%s" m' % str(self.dist_to_wall))
            self.get_logger().info('Distance to front wall: "%s" m' % str(self.front_laser))
            # Logic of move
            self.cmd.linear.x = 0.09
            if self.front_laser < 0.5 or self.front_laser > 15:
                self.flag = 1
            else:
                self.flag = 0

            if self.flag == 1:
                self.cmd.angular.z = 0.55
            
            elif self.flag == 0:
                if self.dist_to_wall < 0.12 or self.dist_to_wall > 10:
                    self.cmd.angular.z = 0.055*2.4
                elif self.dist_to_wall >= 0.12 and self.dist_to_wall < 0.24:
                    self.cmd.angular.z = 0.0
                elif self.dist_to_wall >= 0.24 and self.dist_to_wall < 3:
                    self.cmd.angular.z = -0.055*2.4
            self.get_logger().info('Angular vel z: "%s" rad/s' % str(self.cmd.angular.z))
            # Publishing the cmd_vel values to a Topic        
            self.publisher_.publish(self.cmd)

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    wall_follower = WallFollower()
    # start the communication thread
    spin_thread = Thread(target=rclpy.spin, args=(wall_follower,))
    spin_thread.start()

    # run the send_request() method
    response = wall_follower.send_request()
    wall_follower.resp = response.wallfound
    # Display the message on the console
    wall_follower.get_logger().info(str(response.wallfound)) 

    if response.wallfound == True:
        wall_follower.send_goal()

    # Don't call rclpy.spin() again here, just wait for the thread to handle spinning
    spin_thread.join()
    # Explicity destroys the node
    # wall_follower.destroy_node()
    # shutdown the ROS communication
    # rclpy.shutdown()

if __name__ == '__main__':
    main()