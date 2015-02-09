#!/usr/bin/env python

# Zihan Chen
# 2014-12-04
# Brief: Hybrid Position/Force controller

import rospy
import tf
import math

from std_msgs.msg import String
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Joy
from PyKDL import *
from urdf_parser_py.urdf import URDF

class Hybrid():
    def __init__(self):
        rospy.init_node('hybrid_node')
        self.freq = 100
        self.rate = rospy.Rate(self.freq)  # 100 hz

        # pub
        self.pub_test = rospy.Publisher('/hybrid/test', String)

        # sub
        self.sub_jr3 = rospy.Subscriber('/jr3/wrench', WrenchStamped, self.cb_jr3)
        self.sub_joy = rospy.Subscriber('/spacenav/joy', Joy, self.cb_joy)
        self.sub_enable = rospy.Subscriber('/hybrid/enable', Bool, self.cb_enable)
        self.sub_cmd = rospy.Subscriber('/hybrid/cmd', String, self.cb_cmd)

        # tf 
        self.ler = tf.TransformListener()  # listener
        self.ber = tf.TransformBroadcaster() # broadcaster

        # datas
        self.enabled = False
        self.cmdfrm = Frame()
        self.wrench = Wrench()

        self.cmdtwist = Twist()
        self.urdf = rospy.get_param('/wam/robot_description')
        print self.urdf
        
        self.robot = URDF()
        self.robot = self.robot.from_xml_string(self.urdf)
        self.chain = self.robot.get_chain('base_link',
                                          'cutter_tip_link')

        print self.chain
        pass

    def cb_jr3(self, msg):
        self.wrench.force = Vector(msg.wrench.force.x,
                                   msg.wrench.force.y,
                                   msg.wrench.force.z)
        self.wrench.torque = Vector(msg.wrench.torque.x,
                                    msg.wrench.torque.y,
                                    msg.wrench.torque.z)
        pass

    def cb_joy(self, msg):
        # might want to rotate this twist 
        # self.cmdtwist.vel = Vector(msg.axes[0], msg.axes[1], msg.axes[2])
        # self.cmdtwist.rot = Vector(msg.axes[3], msg.axes[4], msg.axes[5])

        # align twist to be same as base link
        self.cmdtwist.vel = Vector(msg.axes[2], msg.axes[1], -msg.axes[0])
        self.cmdtwist.rot = Vector(msg.axes[5], msg.axes[4], -msg.axes[3])
        # print self.cmdtwist
        pass

    def cb_enable(self, msg):
        self.enabled = msg.data
        if msg.data:
            # sync cmd pos/rot
            rospy.loginfo("enable hybrid force control")
            try:
                (trans, rot) = \
                        self.ler.lookupTransform('wam/base_link',
                                                 'wam/cutter_tip_link',
                                                 rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "tf read error"
            
            self.cmdfrm.p = Vector(trans[0], trans[1], trans[2])
            self.cmdfrm.M = Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3])
            
            print self.cmdfrm
        pass

    def cb_cmd(self, msg):

        pass
        
    def ctrl_hybrid(self):
        # massage cmd frm with force controller
        # z: force control on z axis
        cmdforce = -4  # -4N on z axis
        errforce = cmdforce - self.wrench.force.z()
        print self.wrench.force
        pass

    def run(self):
        while not rospy.is_shutdown():
            msg = String()
            msg.data = 'yeah'
            self.pub_test.publish(msg)

            # update states

            # option 1: direct
            
            # option 2: hybrid

            if self.enabled:
                gain_vel = 0.1
                gain_rot = 0.5
                self.cmdtwist.vel *= gain_vel
                self.cmdtwist.rot *= gain_rot
                # self.cmdfrm.Integrate(self.cmdtwist, self.freq)
                tmptwist = self.cmdfrm.M.Inverse(self.cmdtwist)
                self.cmdfrm.Integrate(tmptwist, self.freq)

                # hybrid
                self.ctrl_hybrid()
                
                cmdtrans = (self.cmdfrm.p.x(), self.cmdfrm.p.y(), self.cmdfrm.p.z())
                cmdrot = self.cmdfrm.M.GetQuaternion()
                self.ber.sendTransform(cmdtrans,
                                       cmdrot,
                                       rospy.Time.now(),
                                       "wam/cmd",
                                       "wam/base_link")
            
            # print cmdTrans
            self.rate.sleep()

if __name__ == '__main__':
    try:
        print ".... Starting Hybrid Position/Force Controller ...."
        ctrl = Hybrid()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
   
