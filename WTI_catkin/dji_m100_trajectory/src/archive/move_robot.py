#! /usr/bin/env python
# Import the Python library for ROS
import rospy
import time
# Import the Twist message
from geometry_msgs.msg import Twist
class MoveRobot():
def __init__(self):
# Initiate a named node
rospy.init_node(’MoveRobot’, anonymous=False)
# tell user how to stop TurtleBot
rospy.loginfo("CTRL + C to stop the turtlebot")
# What function to call when ctrl + c is issued
rospy.on_shutdown(self.shutdown)
# subscribe to the topic published by node random_values
self.new_velocity_sub = rospy.Subscriber(’/new_vel’, Twist,
self.callback_new_velocity)
# Create a Publisher object, will publish on cmd_vel_mux/input/teleop topic
# to which the robot (real or simulated) is a subscriber
self.vel_pub = rospy.Publisher(’/cmd_vel_mux/input/teleop’, Twist,
queue_size=5)
# Creates a var of msg type Twist for velocity
self.vel = Twist()
# publish a topic to notify node pose_monitor of the changed velocity
self.new_velocity_sub = rospy.Publisher(’/change’, Twist, queue_size=1)
# Set a publish velocity rate of in Hz
self.rate = rospy.Rate(5)
def callback_new_velocity(self, msg):
#print "Received Twist msg: ", msg
rospy.loginfo("Received velocity [linear x]%5.2f [angular z]%5.2f",
msg.linear.x, msg.angular.z)
# set velocities as received from the /new_vel topic
self.vel.linear.x = msg.linear.x
self.vel.angular.z = msg.angular.z
# publish the new velocities on the /cmd_vel_mux/input/teleop topic
self.new_velocity_sub.publish(self.vel)
def send_velocity_cmd(self):
self.vel_pub.publish(self.vel)
def shutdown(self):
print "Shutdown!"
# stop TurtleBot
rospy.loginfo("Stop TurtleBot")
self.vel.linear.x = 0.0
self.vel.angular.z = 0.0
self.vel_pub.publish(self.vel)
# makes sure robot receives the stop command prior to shutting down
rospy.sleep(1)
if __name__ == ’__main__’:
try:
controller = MoveRobot()
# keeping doing until ctrl+c
while not rospy.is_shutdown():
# send velocity commands to the robots
controller.send_velocity_cmd()
# wait for the selected mseconds and publish velocity again
controller.rate.sleep()
except:
rospy.loginfo("move_robot node terminated")
