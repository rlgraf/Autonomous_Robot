'''
- rclpy is the ROS2 Client Library for Python that is included with
  any standard installation of ROS2 distribution.

- rclpy provides the standard Application Programming Interface (API)
  for using ROS2 with Python

'''
import rclpy

# node is the main ROS2 class
from rclpy.node import Node

# Twist is the message type we want to send
# Twist is a data structure containing components
# of the linear and angular velocity vectors that
# are used to control the 2D mobile robot

from geometry_msgs.msg import Twist

# this is the topic that is used to send
# velocity control actions to the Gazebo simulation
topic1='cmd_vel'
# rate (frequency) for sending the velocity control messages
# here, we are sending two messages per second
rate_msg=2


def main(args=None):

    # Before using the rclpy API, we must initialize it by using init().
    # This should be done once per process. This function will initialize
    # any global resources that are necessary for the middleware and client libraries.

    
    rclpy.init(args=args)

    # - This is the Twist message that we are sending
    # - The particular selection of the velocity components will cause the 2D mobile robot
    #   to describe a circle

    controlVel = Twist()

    # components of the linear velocity vector
    controlVel.linear.x= 4.0;
    controlVel.linear.y=0.0;
    controlVel.linear.z=0.0;
    # components of the angular velocity vector
    # note that since the robot is in a 2D space, only the z-component of the velocity vector
    # matters, and this is angular velocity around the axis perpendicular to the computer screen
    controlVel.angular.x=0.0;
    controlVel.angular.y=0.0;
    controlVel.angular.z=8.0;

    # create a node
    TestNode=Node("test_node")
    #create a publisher
    publisher=TestNode.create_publisher(Twist,topic1, 1)
    # create a period/rate for sending messages
    # here, we are sending "rate_msg" messages per second
    rate= TestNode.create_rate(rate_msg)

    # here, we send messages
    while rclpy.ok():
        # print the message
        print("Sending control message")
        # publish
        publisher.publish(controlVel)

        # spin
        # Execute one item of work or wait until a timeout expires.
        # this is a non blocking statement in contrast to rclpy.spin()
        rclpy.spin_once(TestNode)
        # sleep for the specified period
        rate.sleep()

    # destroy the node
    TestNode.destroy_node()
    # shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()