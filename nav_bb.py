#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import UInt16
import pickle
from opcua import ua, Client
import time
import logging
import sys
import tf2_ros
import roslaunch

#logging.basicConfig(level=logging.INFO)
#_logger = logging.getLogger('opcua')


class NavTest():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 1)

        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)

        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED',
                       'ABORTED', 'REJECTED', 'PREEMPTING', 'RECALLING',
                       'RECALLED', 'LOST']

        # Set up the goal locations. Poses are defined in the map frame.
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        locations = dict()

        locations['1'] = Pose(Point(-1.365, -0.464, 0.000),
                              Quaternion(0.000, 0.000, -0.806, 0.591))

        locations['2'] = Pose(Point(-1.702, -1.068, 0.000),
                              Quaternion(0.000, 0.000, -0.148, 0.989))

        locations['3'] = Pose(Point(0.373, -0.149, 0.000),
                              Quaternion(0.000, 0.000, -0.791, 0.611))

        #locations['4'] = Pose(Point(4.4076, 0.815, 0.000),
         #                     Quaternion(0.000, 0.000, 0.468, 0.884))

        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")

        rospy.loginfo("Starting navigation test")

        def move(i):
            # Get the next location in the current sequence
            location = 'Position' + str(i)
	
            # Set up the next goal location
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = locations[str(i)]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            # Let the user know where the robot is going next
            rospy.loginfo("Going to: " + str(location))	
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)

            # Allow 5 minutes to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))
            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                else:
                    rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))

        # Identify the AR code(get V4 from PLC)
        def runV4():
            flag = 0
            Stop = False
            while Stop == False:
               if flag == 0:
                    try:
                        client = Client("opc.tcp://192.168.1.10:4840")
                        client.set_user('admin')
                        client.set_password('d0a904e3')
                        client.set_security_string("Basic256Sha256,SignAndEncrypt,certificate-example.der,private-key-example.pem")
                        print(1)
                        client.connect()
                        print(2)
                        flag = 1
                    except:
                        time.sleep(1)
                        continue
                    try:
                        #print(2)
                        node4 = client.get_node("ns=5;s=Arp.Plc.Eclr/DI_V4")
                        V4=node4.get_value()
                        print(V4)
                        while not V4 == True:
                            rospy.sleep(1)
                            V4 = node4.get_value()
                        # Move to positionA
                        print('move1')
                        move(1)
                        move(2)
                        Stop=True
                    finally:
                        flag=0
                        client.disconnect()

        runV4()
        print('runV4finished')
        #Change V2 from PLC via opcua
        def runV2():
            flag = 0
            Stop = False
            while Stop == False:
               if flag == 0:
                    try:
                        client = Client("opc.tcp://192.168.1.10:4840")
                        client.set_user('admin')
                        client.set_password('d0a904e3')
                        client.set_security_string("Basic256Sha256,SignAndEncrypt,certificate-example.der,private-key-example.pem")
                        client.connect()
                        flag = 1

                    except:
                        time.sleep(1)
                        continue
                    try:
                        #print(2)
                        node2 = client.get_node("ns=5;s=Arp.Plc.Eclr/DI_V2")
                        node2.set_value(ua.DataValue(True))

                        Stop=True
                        print(node2.get_data_value())
                    finally:
                        flag=0
                        client.disconnect()

        transformData = TransformStamped()
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
            
        rate = rospy.Rate(10.0)

        def starttotrace():


            print('starttotrace111')
            while not rospy.is_shutdown():
                print('whilenotrospyshutdown')
                try:
                    transformData = tfBuffer.lookup_transform('usb_cam', 'ar_marker_3', rospy.Time(0))
                    if transformData.child_frame_id == 'ar_marker_3':
                        print('V2')
                        runV2()
                        break
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print('pass')
                    pass
                rate.sleep()

        # Change V2 from PLC via opcua
        # Get V3 from PLC via opcua
        # start to trace
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        tracking_launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/cdhawer/catkin_ws/src/tb3_localization/launch/follower.launch"])

        def goback():
            self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

            # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ

            r = rospy.Rate(10);

            # Twist is a datatype for velocity

            move_cmd = Twist()

            # let's go forward at 0.2 m/s

            move_cmd.linear.x = -0.10

            # let's turn at 0 radians/s

            move_cmd.angular.z = 0


            for x in range(0,25):
                self.cmd_vel.publish(move_cmd)
                r.sleep()

            move_cmd.linear.x = 0

            move_cmd.angular.z = 0


            for x in range(0,10):
                self.cmd_vel.publish(move_cmd)
                r.sleep()

        def setpose():
            publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
            # Creating the message with the type PoseWithCovarianceStamped
            rospy.loginfo(
                "This node sets the turtlebot's position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")
            start_pos = PoseWithCovarianceStamped()
            # filling header with relevant information
            start_pos.header.frame_id = "map"

            # filling payload with relevant information gathered from subscribing
            # to initialpose topic published by RVIZ via rostopic echo initialpose
            start_pos.pose.pose.position.x = 0.79
            start_pos.pose.pose.position.y = -1.93
            start_pos.pose.pose.position.z = 0.0

            start_pos.pose.pose.orientation.x = 0.0
            start_pos.pose.pose.orientation.y = 0.0
            start_pos.pose.pose.orientation.z = -0.16
            start_pos.pose.pose.orientation.w = 0.99

            start_pos.pose.covariance[0] = 0.25
            start_pos.pose.covariance[7] = 0.25
            start_pos.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            start_pos.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            start_pos.pose.covariance[35] = 0.06853891945200942

            rospy.loginfo(start_pos)
            rospy.sleep(2)
            publisher.publish(start_pos)
            rospy.sleep(2)
            publisher.publish(start_pos)

        def runV3():
            flag = 0
            signal = 0
            Stop = False
            while Stop == False:
               if flag == 0:
                    try:

                     client = Client("opc.tcp://192.168.1.10:4840")
                     client.set_user('admin')
                     client.set_password('d0a904e3')
                     client.set_security_string("Basic256Sha256,SignAndEncrypt,certificate-example.der,private-key-example.pem")
                     client.connect()
                     flag = 1

                    except:
                        time.sleep(1)
                        continue
                    try:
                        #print(2)
                        node3 = client.get_node("ns=5;s=Arp.Plc.Eclr/DI_V3")
                        V3=node3.get_value()

                        if V3 == True:
                            tracking_launch.shutdown()
                            setpose()
                            goback()
                            move(3)
                            Stop=True
                        else:
                            if signal==0:
                                tracking_launch.start()
                                rospy.sleep(10)
                                print ('starttotrace')
                                starttotrace()
                                signal=1
                            else:
                                pass
                    finally:
                        flag=0
                        client.disconnect()

        # Go back to the starting point
        runV3()
        def runfinal():
            flag = 0
            Stop = False
            while Stop == False:
               if flag == 0:
                    try:
                        client = Client("opc.tcp://192.168.1.10:4840")
                        client.set_user('admin')
                        client.set_password('d0a904e3')
                        client.set_security_string("Basic256Sha256,SignAndEncrypt,certificate-example.der,private-key-example.pem")
                        client.connect()
                        flag = 1

                    except:
                        time.sleep(1)
                        continue
                    try:
                        #print(2)
                        nodefinal1 = client.get_node("ns=5;s=Arp.Plc.Eclr/DI_V2")
                        nodefinal1.set_value(ua.DataValue(False))
                        nodefinal2 = client.get_node("ns=5;s=Arp.Plc.Eclr/DI_V4")
                        nodefinal2.set_value(ua.DataValue(False))
                        Stop=True
                    finally:
                        flag=0
                    client.disconnect()
        runfinal()

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    while True:
        try:
            NavTest()
            print('this loop is finished')

        except rospy.ROSInterruptException:
            rospy.loginfo("AMCL navigation test finished.")
    rospy.spin()
