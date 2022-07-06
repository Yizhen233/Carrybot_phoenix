#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from opcua import ua, Client
import time
import logging
import sys



def runV2():
    flag = 0
    Stop = False
    while Stop == False:
        if flag == 0:
            try:
                client = Client("opc.tcp://192.168.1.10:4840")
                client.set_user('admin')
                client.set_password('d0a904e3')
                client.set_security_string(
                    "Basic256Sha256,SignAndEncrypt,certificate-example.der,private-key-example.pem")
                client.connect()
                flag = 1

            except:
                time.sleep(1)
                continue
            try:
                # print(2)
                node2 = client.get_node("ns=5;s=Arp.Plc.Eclr/TEST00")
                V2=node2.get_value()
                print (node2.get_value())
                while V2 == False:
                    print(1)
                Stop = True
            finally:
                flag = 0
            client.disconnect()

if __name__ == '__main__':

    rospy.init_node('node_listener')
    transformData = TransformStamped()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            transformData = tfBuffer.lookup_transform('usb_cam', 'ar_marker_3', rospy.Time(0))
            a=transformData.child_frame_id()
            if a == '3':
                runV2()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        rate.sleep()

