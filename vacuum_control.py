#!/usr/bin/python

import cv2
import numpy as np
import rospy

from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import tf




class vacuum_control:
    def __init__(self):
 
        #subscribers
        self.vacuum_sub = rospy.Subscriber("/turtlebot/swiftpro/vacuum_gripper/pump_state", Bool, self.get_vacuum)
        # self.vacuum_sub = rospy.Subscriber("/set_vacuum", Bool, self.set_vacuum)
        self.vacuum_state = None


    def vacuum_on(self):
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')
        try:
            set_pump = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
            response = set_pump(True)
            rospy.loginfo("Vacuum ON: %s", response.success)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    #Turn Vaccum Off to Place object
    def vacuum_off(self):
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')
        try:
            set_pump = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
            response = set_pump(False)
            rospy.loginfo("Vacuum OFF: %s", response.success)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
    
   
    def get_vacuum(self,msg:Bool):
        self.vacuum_state = msg.data    
    
    # def set_vacuum(self.msg:Bool):





if __name__ == "__main__":
    
    rospy.init_node('vacuum_control')
    node = vacuum_control()
    # node.vacuum_on()
    rospy.spin()
    