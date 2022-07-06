#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def initial_pos_pub():
    publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.init_node('initial_pos_pub', anonymous=True)
    #Creating the message with the type PoseWithCovarianceStamped
    rospy.loginfo("This node sets the turtlebot's position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")
    start_pos = PoseWithCovarianceStamped()
    #filling header with relevant information
    start_pos.header.frame_id = "map"

    #filling payload with relevant information gathered from subscribing
    # to initialpose topic published by RVIZ via rostopic echo initialpose
    start_pos.pose.pose.position.x = 0.373
    start_pos.pose.pose.position.y = -0.149
    start_pos.pose.pose.position.z = 0.0

    start_pos.pose.pose.orientation.x = 0.0
    start_pos.pose.pose.orientation.y = 0.0
    start_pos.pose.pose.orientation.z = -0.791
    start_pos.pose.pose.orientation.w = 0.611

    start_pos.pose.covariance[0] = 0.25
    start_pos.pose.covariance[7] = 0.25
    start_pos.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    start_pos.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    start_pos.pose.covariance[35] = 0.06853891945200942

    rospy.loginfo(start_pos)
    rospy.sleep(2)
    publisher.publish(start_pos)
    rospy.sleep(2)
    publisher.publish(start_pos)


if __name__ == '__main__':
    try:

        initial_pos_pub()

    except rospy.ROSInterruptException:
        pass
