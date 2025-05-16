#!/usr/bin/python

import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose,PoseStamped
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import JointState, Imu 
from std_msgs.msg import Float64MultiArray ,Bool
from tf.broadcaster import TransformBroadcaster
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from visualization_msgs.msg import Marker
from visualize import publish_goal_marker,publish_trajectory_marker,publish_goal_pose,publish_current_pose,publish_robot_pose
from manipulator import Manipulator
from task import Position3D,JointLimit,Configuration3D,Orientation3D

class Manipulator_Control:
    def __init__(self) -> None:


        
        # joint state subscriber
        self.js_sub = rospy.Subscriber("/turtlebot/joint_states", JointState, self.joint_state_callback)
        self.position_sub =  rospy.Subscriber("/swift_pro/goal_position", PoseStamped, self.goal_callback)
        self.odom_sub =  rospy.Subscriber("/turtlebot/kobuki/odom_ground_truth", Odometry, self.odom_callback)

        # joint position publisher
        self.vel_pub = rospy.Publisher("/turtlebot/swiftpro/joint_velocity_controller/command", Float64MultiArray, queue_size=20)
        self.base_vel_pub = rospy.Publisher("/turtlebot/kobuki/commands/velocity", Twist, queue_size=10)
        self.robot_pub = rospy.Publisher("/robot_pose", PoseStamped, queue_size=10)
        self.ee_pub = rospy.Publisher("/ee_pose", PoseStamped, queue_size=10)
        self.goal_pub = rospy.Publisher("/goal_pose", PoseStamped, queue_size=10)
        self.trajectory_pub = rospy.Publisher("/trajectory_marker", Marker, queue_size=10)
        self.manipulator_state_pub = rospy.Publisher("/swift_pro/goal_reach", Bool, queue_size=10)
        self.goal_position = None
        self.end_point = None
        d = [0.108, 0, 0, 0.0565]
        a = [0.0132, 0.142, 0.1588, 0.0722]
        theta = [0, 0, 0, 0]
        alpha = [0, 0, 0, 0]
        revolute = [True,True,True,True]
        self.trajectory = []
        self.trajectory_limit = []
        self.robot = Manipulator(d,theta,a,alpha,revolute)
        self.error = None
        self.robot_position = None
        self.last_time = rospy.Time.now()
        self.q = None
        

        self.tasks = [
                    JointLimit("Joint 1 Limit",np.array([-1.571, 1.571]).reshape(2,1), np.array([0.05, 0.1]).reshape(2,1),1),
                    JointLimit("Joint 2 Limit",np.array([-1.571, 0.05]).reshape(2,1),np.array([0.05, 0.1]).reshape(2,1),2),
                    JointLimit("Joint 3 Limit",np.array([-1.571, 0.05]).reshape(2,1), np.array([0.05, 0.1]).reshape(2,1),3),
                    JointLimit("Joint 4 Limit",np.array([-1.571, 1.571]).reshape(2,1), np.array([0.05, 0.1]).reshape(2,1),4),

                    Position3D("End-Effector Configuration",np.array([ 0.250, -0.050 , -0.30 ]),-1)]
                    # Configuration3D("End-Effector Configuration",np.array([0.157, -0.150 , -0.250,-1]),-1)]

        rospy.Timer(rospy.Duration(0.1), self.controller)
        # rospy.Timer(rospy.Duration(0.1), self.set_random_goal)
   
    def controller(self,event=None):
        
        if self.robot_position is None:
            return
        
        
        dof = self.robot.dof
        p  = np.eye(dof)
        dq = np.zeros((dof,1))
        self.robot.update(self.q,self.robot_position)
        current_position = self.robot.getEETransform()[0:3,3].reshape(3,1)
        publish_current_pose(self.robot.getEETransform(),self.ee_pub)
        # print(f'current position:{current_position[0][0]:.3f},{current_position[1][0]:.3f},{current_position[2][0]:.3f}')
        if self.goal_position is None:
            return
        self.__sent_state__(False)     
        rospy.loginfo(f"[Z-MOVE] Error norm: {np.linalg.norm(self.tasks[-1].getError()):.4f}")
        rospy.loginfo(f"[Z-MOVE] Desired Z: {self.tasks[-1].sigma_d[2]:.4f}, Current Z: {current_position[2, 0]:.4f}")


        # print(f'robot_position before:{self.robot_position[2,0]:.3f}')
        for indx,task in enumerate(self.tasks):

            task.update(self.robot)
            
            if task.get_activate_value()!=0:
                
                J = task.getJacobian()

                J_bar = J @ p
                error = np.asarray(task.getError(), dtype=np.float64)
                # print(f'error:{np.linalg.norm(error):.3f}')
                # print(J_bar)
                dq += self.DLS(J_bar) @ (error - J @ dq)
                # print(dq)
                p = p - np.linalg.pinv(J_bar) @ J_bar
                self.error = error
            else:
                dq = dq
                p = p
            if task.get_activate_value() == -1:
                self.trajectory_limit.append(True)
            else:
                self.trajectory_limit.append(False)      
        
        self.__sent_velocity__(dq)
        current_position = self.robot.getEETransform()[0:3,3].reshape(3,1)
        # print(f'current_position theta:{current_position[2,0]:.3f}')
        # print(f'robot_position: {self.robot.eta[0,0]:.3f} {self.robot.eta[1,0]:.3f} {self.robot.eta[2,0]:.3f}')
        self.trajectory.append(current_position)

        publish_goal_pose(self.tasks[-1].sigma_d,self.goal_pub,current=current_position)
        publish_current_pose(self.robot.getEETransform(),self.ee_pub)
        # print(f'goal:{self.tasks[-1].sigma_d[0]:.3f} {self.tasks[-1].sigma_d[1]:.3f} {self.tasks[-1].sigma_d[2]:.3f}')
        # print(f'error:{np.linalg.norm(self.tasks[-1].getError()):.3f}')
        # print(np.linalg.norm(current_position-self.goal_position[:3]))
        if np.linalg.norm(self.tasks[-1].getError())<0.01:
            # print(f'error:{np.linalg.norm(self.tasks[-1].getError()):.3f}')
            # self.end_point = np.array([self.goal_position[0], self.goal_position[1], -0.15])
            # # self.tasks[-1].setDesired(np.array([self.goal_position[0], self.goal_position[1], -0.15]))
            # # print(f'goal:{self.tasks[-1].sigma_d[0]:.3f} {self.tasks[-1].sigma_d[1]:.3f} {self.tasks[-1].sigma_d[2]:.3f}')
            # if np.linalg.norm(self.tasks[-1].getError())<0.001:
                # print(f'error:{np.linalg.norm(self.tasks[-1].getError()):.3f}')
                print(f'goal reach!')
                dq = np.zeros((dof,1))
                self.__sent_velocity__(dq)
                self.goal_position = None
                self.end_point = None
                self.__sent_state__(True)
                return
        self.__sent_state__(False)        

        if len(self.trajectory)>2:
            publish_trajectory_marker(self.trajectory,self.trajectory_limit,self.trajectory_pub)

    def joint_state_callback(self, msg:JointState):
        # pass
        if 'passive' not in msg.name[0] and 'swiftpro' in msg.name[0]:
            joint = np.array(msg.position[:4])
            self.robot.q[2:] = joint.reshape(-1,1)
            self.q = joint.reshape(-1,1)
            # print(f'real joint: {joint[0]:.3f} {joint[1]:.3f} {joint[2]:.3f} {joint[3]:.3f}')
    
    def goal_callback(self, msg:PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        # z = -0.25

        # q = quaternion_from_euler(0, 0, 1.57)  # 1.57 rad = 90 deg
        # goal.pose.orientation.x = q[0]
        # goal.pose.orientation.y = q[1]
        # goal.pose.orientation.z = q[2]
        # goal.pose.orientation.w = q[3]
        
        #if self.goal_position is not None:
        #    return
        if self.goal_position is not None:
            # Check if the new goal is significantly different
            new_goal = np.array([x, y, z])
            if np.linalg.norm(new_goal - self.goal_position) < 0.005:
                return

        self.goal_position = np.array([x, y, z])
        self.tasks[-1].setDesired(np.array([x, y, z]))
        self.last_time = rospy.Time.now()
        self.trajectory = []
        self.trajectory_limit = []

    

    def odom_callback(self, msg: Odometry):
        # Update eta with base pose
        self.robot.robot_pose[0, 0] = msg.pose.pose.position.x
        self.robot.robot_pose[1, 0] = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        self.robot.robot_pose[2, 0] = yaw
        self.robot_position = np.zeros((3,1))
        self.robot_position[0,0] = msg.pose.pose.position.x
        self.robot_position[1,0] = msg.pose.pose.position.y
        self.robot_position[2,0] = yaw
        publish_robot_pose(self.robot.robot_pose,self.robot_pub)
        # print(f'yaw: {yaw:.3f}')
       

    def set_random_goal(self,event=None):
        dt = (rospy.Time.now() - self.last_time).to_sec()
        if self.error is None:
            return 
        current_position = self.robot.getEETransform()[0:3, 3].reshape(3, 1)
        rospy.loginfo(f"[Z-POS] Current EE Z = {current_position[2,0]:.3f}")

        # Get the desired goal position from the Configuration3D task
        goal_position = self.tasks[-1].sigma_d[:3].reshape(3, 1)  # Only x, y, z
        # Compute the distance to the goal
        distance_to_goal = np.linalg.norm(current_position - goal_position)
        if distance_to_goal < 0.01 or dt > 60 :
            # dq = np.zeros((self.robot.dof,1))
            # self.__sent_velocity__(dq)
            x = np.random.uniform(-0.50, 0.50)
            y = np.random.uniform(-0.50, 0.50)
            z = np.random.uniform(-0.300, -0.200)
            theta = np.random.uniform(-1.0, 1.0)
            self.tasks[-1].setDesired(np.array([x, y, z,theta]))
            
            self.last_time = rospy.Time.now()
            self.trajectory = []
            self.trajectory_limit = []
            return  

    def __sent_velocity__(self,dq):
        msg = Float64MultiArray()
        msg.data = dq[2:].flatten().tolist()
        self.vel_pub.publish(msg)

        base_msg = Twist()  
        
        base_msg.linear.x = dq[1,0]
        base_msg.angular.z = -dq[0,0]
        # print(f'angular v:{-dq[0,0]:.3f}:, linear v:{dq[1,0]:.3f}')
        # print(f'angular v:{-dq[0,0]:.3f}, linear v:{dq[1,0]:.3f},joint1:{dq[2,0]:.3f}, joint2:{dq[3,0]:.3f}, joint3:{dq[4,0]:.3f}, joint4:{dq[5,0]:.3f}')
        self.base_vel_pub.publish(base_msg)

      
    def DLS( self, A, damping_factor=0.1):

        J = A.T @ np.linalg.inv(A @ A.T + damping_factor**2 * np.eye(A.shape[0]))
        return J
    
    def __sent_state__(self,value):
        msg = Bool()
        msg.data = value
        self.manipulator_state_pub.publish(msg)
    

    



if __name__ == '__main__':

    rospy.init_node("SwiftPro_Control")

    robot = Manipulator_Control()

    rospy.spin()

    