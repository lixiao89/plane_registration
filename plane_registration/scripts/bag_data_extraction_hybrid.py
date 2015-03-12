#!/usr/bin/env python

# Xiao Li
# 2015-2-07
# brief: to extract data from bag files to text files for conceptual analysis in matlab

import rospy
import tf
import math

from std_msgs.msg import String
from geometry_msgs.msg import Pose
from cisst_msgs.msg import vctDoubleVec
from sensor_msgs.msg import CameraInfo
from PyKDL import *

class DataExtraction():
    def __init__(self):
        rospy.init_node('data_extraction_node')
        self.freq = 100
        self.rate = rospy.Rate(self.freq)  # 100 hz

        # sub
        self.sub_ft = rospy.Subscriber('/logger/MsrFT', vctDoubleVec, self.cb_ft)
        self.sub_msrSE3 = rospy.Subscriber('/logger/MsrSE3', Pose, self.cb_msrSE3)
        self.sub_time = rospy.Subscriber('/stereo/left/camera_info',CameraInfo,self.cb_cameraInfo)

        self.ee_position = Vector()
        self.ee_orientation = Vector()
        self.ee_w = 0
        self.ee_force = Vector()
  
        self.f = open('bag_data', 'w')
        self.time = rospy.Time()
        pass

    def cb_msrSE3(self, msg):
        self.ee_position = Vector(msg.position.x,
                                  msg.position.y,
                                  msg.position.z)

        self.ee_orientation = Vector(msg.orientation.x,
                                     msg.orientation.y,
                                     msg.orientation.z)
        self.ee_w = msg.orientation.w
        pass

    def cb_ft(self, msg):
        self.ee_force = Vector(msg.data[0],
                               msg.data[1],
                               msg.data[2])

        pass

    def cb_cameraInfo(self, msg):
        self.time = msg.hearder.stamp
        pass

    def run(self):
        while not rospy.is_shutdown():

            self.f.write(str(self.time.to_sec()) + ","
                         + str(self.ee_position[0]) + "," 
                         + str(self.ee_position[1])+ ","
                         + str(self.ee_position[2])+ ","
                         + str(self.ee_orientation[0])+ ","
                         + str(self.ee_orientation[1])+ ","
                         + str(self.ee_orientation[2])+ ","
                         + str(self.ee_w)+ ","
                         + str(self.ee_force[0])+ ","
                         + str(self.ee_force[1])+ ","
                         + str(self.ee_force[2])+ "," + '\n')
           
            self.rate.sleep()



# Program starting point
if __name__ == '__main__':
    try:
        print "start data extraction"
        data_ext = DataExtraction()
        data_ext.run()
    except rospy.ROSInterruptException:
        pass
   
