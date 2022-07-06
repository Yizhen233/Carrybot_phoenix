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





def callback(data):
    global stop1
    if len(data.markers) == 0:
        stop1 = 1
        print ('stop1')
    else:
        stop1 = 0
        print ('stop0')
if __name__=='__main__':
    sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
    while not rospy.is_shutdown():
        if stop1 == 1:
            print('nothinginside')
            rospy.sleep(10)
            break
        else:
            rospy.sleep(2)
            print('somethinginside')
    rospy.spin()