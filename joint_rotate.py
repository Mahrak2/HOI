#!/usr/bin/python

import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose,PoseStamped
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
from tf.broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from visualization_msgs.msg import Marker
from visualize import publish_goal_marker,publish_trajectory_marker,publish_goal_pose,publish_current_pose,publish_robot_pose
from manipulator import Manipulator
from task import Position3D,JointLimit,Configuration3D,Orientation3D

class joint_rotate:
    def __init__(self) -> None:


        
        # joint state subscriber
        self.js_sub = rospy.Subscriber("/turtlebot/joint_states", JointState, self.joint_state_callback)
        # joint position publisher
        self.vel_pub = rospy.Publisher("/turtlebot/swiftpro/joint_velocity_controller/command", Float64MultiArray, queue_size=20)
        self.q = None
       

        rospy.Timer(rospy.Duration(0.1), self.controller)
        # rospy.Timer(rospy.Duration(0.1), self.set_random_goal)
   
    def controller(self,event=None):
        
        if self.q is None:
            return 
        dq = np.zeros((4,1))
        error = 1.57 - self.q[0]
        dq[0] = error
        
        self.__sent_velocity__(dq)
        

    def joint_state_callback(self, msg:JointState):
        # pass
        
        if 'passive' not in msg.name[0] and 'swiftpro' in msg.name[0]:
            joint = np.array(msg.position[:4])
            self.q = joint.reshape(-1,1)
            print(f'real joint: {joint[0]:.3f} {joint[1]:.3f} {joint[2]:.3f} {joint[3]:.3f}')
    
   

    

   

    def __sent_velocity__(self,dq):
        msg = Float64MultiArray()
        msg.data = dq[:].flatten().tolist()
        self.vel_pub.publish(msg)

      
      
    

    



if __name__ == '__main__':

    rospy.init_node("joint_rotate")

    robot = joint_rotate()

    rospy.spin()

    





