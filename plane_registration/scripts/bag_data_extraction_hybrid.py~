#!/usr/bin/env python

# Xiao Li
# 2015-2-07
# brief: to extract data from bag files to text files for conceptual analysis in matlab

import rospy
import tf
import math

from std_msgs.msg import String
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Pose
from cisst_msgs.msg import vctDoubleVec
from PyKDL import *
from urdf_parser_py.urdf import URDF

class DataExtraction():
    def __init__(self):
        rospy.init_node('data_extraction_node')
        self.freq = 100
        self.rate = rospy.Rate(self.freq)  # 100 hz

        # sub
        self.sub_ft = rospy.Subscriber('/logger/MsrFT', vctDoubleVec, self.cb_ft)
        self.sub_msrSE3 = rospy.Subscriber('/logger/MsrSE3', Pose, self.cb_msrSE3)

        self.ee_position = Vector()
        self.ee_force = Vector()

        self.f = open('bag_data', 'w')
        pass

    def cb_msrSE3(self, msg):
        self.ee_position = Vector(msg.position.x,
                                  msg.position.y,
                                  msg.position.z)

        pass

    def cb_ft(self, msg):
        self.ee_force = Vector(msg.data[0],
                               msg.data[1],
                               msg.data[2])

        pass

    def run(self):
        while not rospy.is_shutdown():

            self.f.write(str(self.ee_position[0]) + "," 
                         + str(self.ee_position[1])+ ","
                         + str(self.ee_position[2])+ ","
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
   
