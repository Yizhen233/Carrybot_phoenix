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
from ar_track_alvar_msgs.msg import AlvarMarkers
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

        locations['1'] = Pose(Point(-0.3487, -1.463, 0.000),
                              Quaternion(0.000, 0.000, -0.15, 0.988))

        locations['11'] = Pose(Point(-0.296, -1.49, 0.000),
                              Quaternion(0.000, 0.000, -0.15, 0.988))
        locations['12'] = Pose(Point(0.28, -1.68, 0.000),
                              Quaternion(0.000, 0.000, -0.158, 0.988))
        locations['13'] = Pose(Point(0.689, -1.827, 0.000),
                              Quaternion(0.000, 0.000, -0.162, 0.987))
        locations['14'] = Pose(Point(1.058, -1.946, 0.000),
                             Quaternion(0.000, 0.000, -0.15, 0.988))

        locations['2'] = Pose(Point(1.41, -2.07, 0.000),
                              Quaternion(0.000, 0.000, -0.16, 0.988))

        locations['3'] = Pose(Point(0.792, -0.32, 0.000),
                              Quaternion(0.000, 0.000, -0.82, 0.57))

        locations['4'] = Pose(Point(-0.7, -0.61, 0.000),
                              Quaternion(0.000, 0.000, 0.986, 0.165))
        #locations['4'] = Pose(Point(),
        #                     Quaternion(0.000, 0.000, 0.998, 0.067))

        locations['5'] = Pose(Point(-0.65, -1.35, 0.000),
                              Quaternion(0.000, 0.000, -0.132, 0.991))


        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")

        # Let the claw open
        cmd_msg = UInt16()
        pub = rospy.Publisher('servo', UInt16, queue_size=1)
        print (70)
        cmd_msg.data = 90
        pub.publish(cmd_msg)
        rospy.sleep(3)
        pub.publish(cmd_msg)

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

        #Get V1 from PLC via opcua
        def runV1():
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
                        node1 = client.get_node("ns=5;s=Arp.Plc.Eclr/DI_V1")
                        print(3)
                        V1=node1.get_value()
                        while not V1 == True:
                            rospy.sleep(1)
                            print(node1.get_value())
                            V1 = node1.get_value()
                        move(1)
                        print('move1')
                        Stop=True
                    finally:
                        flag=0
                    client.disconnect()
        #Move to positionA
        runV1()


        #Get V2 from PLC via opcua
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
                        node4 = client.get_node("ns=5;s=Arp.Plc.Eclr/DI_V4")
                        node4.set_value(ua.DataValue(True))
                        print(node4.get_value())
                        V2=node2.get_value()
                        while not V2 == True:
                            rospy.sleep(1)
                            V2 = node2.get_value()
                        Stop = True
                    finally:
                        flag=0
                    client.disconnect()
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

        goback()

        # Close the claw
        cmd_msg.data = 0
        pub.publish(cmd_msg)
        rospy.sleep(3)

        #Change V3 from PIC via opcua
        def runV3():
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
                        node3 = client.get_node("ns=5;s=Arp.Plc.Eclr/DI_V3")
                        node3.set_value(ua.DataValue(True))
                        Stop=True
                        print(node3.get_value())
                    finally:
                        flag=0
                    client.disconnect()

        transformData = TransformStamped()
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(10.0)

        # Get V2 from PLC via opcua
        def getV2():

                print('getV2')
                while not rospy.is_shutdown():
                    print('whilenotrospyshutdown')
                    try:
                        transformData = tfBuffer.lookup_transform('usb_cam', 'ar_marker_3', rospy.Time(0))
                        if transformData.child_frame_id == 'ar_marker_3':
                            print('big')
                            runV2()
                            break
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        pass
                    try:
                        transformData = tfBuffer.lookup_transform('usb_cam', 'ar_marker_2', rospy.Time(0))
                        if transformData.child_frame_id == 'ar_marker_2':
                            print('small')
                            break
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        pass

                    rate.sleep()

        getV2()

        # Move to PositionB
        def goforward():
            self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

            # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ

            r = rospy.Rate(10);

            # Twist is a datatype for velocity

            move_cmd = Twist()

            # let's go forward at 0.2 m/s

            move_cmd.linear.x = 0.11

            # let's turn at 0 radians/s

            move_cmd.angular.z = 0


            for x in range(0,150):
                self.cmd_vel.publish(move_cmd)
                r.sleep()

            move_cmd.linear.x = 0

            move_cmd.angular.z = 0


            for x in range(0,5):
                self.cmd_vel.publish(move_cmd)
                r.sleep()

        #goforward()
        move(11)
        move(12)
        #move(13)
        #move(14)
        move(2)
        runV3()



        def goround1():
            self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

            # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ

            r = rospy.Rate(10);

            # Twist is a datatype for velocity

            move_cmd = Twist()

            # let's go forward at 0.2 m/s

            move_cmd.linear.x = 0.18

            # let's turn at 0 radians/s

            move_cmd.angular.z = 0.6

            # as long as you haven't ctrl + c keeping doing...


            for x in range(0,75):
                self.cmd_vel.publish(move_cmd)
                r.sleep()

            move_cmd.linear.x = 0

            move_cmd.angular.z = 0


            for x in range(0,10):
                self.cmd_vel.publish(move_cmd)
                r.sleep()

        def goround2():
            self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)

            # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ

            r = rospy.Rate(10);

            # Twist is a datatype for velocity

            move_cmd = Twist()

            # let's go forward at 0.2 m/s

            move_cmd.linear.x = 0.18

            # let's turn at 0 radians/s

            move_cmd.angular.z = 0.6

            # as long as you haven't ctrl + c keeping doing...


            for x in range(0,50):
                self.cmd_vel.publish(move_cmd)
                r.sleep()

            move_cmd.linear.x = 0

            move_cmd.angular.z = 0


            for x in range(0,10):
                self.cmd_vel.publish(move_cmd)
                r.sleep()


        stop1 = 1
        # wait for people to take out the box
        rospy.sleep(5)
        goround1()

        move(4)
        goround2()
        # Go back to PositionA
        move(5)

        # Open the claw
        cmd_msg.data = 90
        pub.publish(cmd_msg)
        rospy.sleep(3)



        # Go back to the starting point


        move(3)
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
                        nodefinal1 = client.get_node("ns=5;s=Arp.Plc.Eclr/DI_V4")
                        nodefinal1.set_value(ua.DataValue(False))
                        nodefinal2 = client.get_node("ns=5;s=Arp.Plc.Eclr/DI_V3")
                        nodefinal2.set_value(ua.DataValue(False))
                        Stop=True
                    finally:
                        flag=0
                    client.disconnect()
        runfinal()

    def update_initial_pose(self, initial_pose):
            self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    while True:
        try:
            stop = 1
            NavTest()
            print('this loop is finished')

        except rospy.ROSInterruptException:
            rospy.loginfo("AMCL navigation test finished.")

    rospy.spin()








