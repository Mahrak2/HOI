#!/usr/bin/python

import numpy as np

import math


class Manipulator:                                  
    def __init__(self, d, theta, a, alpha, revolute):

        
        self.d = d
        self.a = a
        self.theta = theta
        self.alpha = alpha
        self.revolute = revolute
        self.revoluteExt =[True,False] + self.revolute   
        # self.dof= len(self.revolute)
        self.dof = len(self.revoluteExt) 
        self.q = np.zeros((self.dof,1)) 
        self.robot_pose = np.zeros((3,1))

        
        

    def update(self, q, robot_position):
     
 

        self.q[2:] = q
        self.joints_pos = q.reshape(1,-1)
        self.robot_pose = np.zeros((3,1))
        self.robot_pose[0,0] = robot_position[0,0]
        self.robot_pose[1,0] = robot_position[1,0]
        self.robot_pose[2,0] = robot_position[2,0]
        for i in range(2,len(self.revoluteExt)):
            if self.revolute[i - 2]:
                self.theta[i - 2] = self.q[i, 0]
            else:
                self.d[i - 2] = self.q[i, 0]
        
        
    
        Tb = np.array([[np.cos(self.robot_pose[2,0]), -np.sin(self.robot_pose[2,0]), 0, self.robot_pose[0,0]],\
                       [np.sin(self.robot_pose[2,0]), np.cos(self.robot_pose[2,0]),  0, self.robot_pose[1,0]],\
                       [0, 0, 1, 0],\
                       [0, 0, 0, 1]]) 
        

        
        self.T = self.kinematics(self.d, self.theta,self.a,self.alpha,0,0, tb = Tb)

    def kinematics(self, d, theta, a, alpha, robot_r, robot_x,tb = np.eye(4)):
        
        
        
        self.robot_r = robot_r
        self.robot_x = robot_x
        r0 =  np.array([[np.cos(robot_r), - np.sin(robot_r),0,robot_x],
                        [np.sin(robot_r), np.cos(robot_r),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]).reshape(4,4)
        
        
        t0 = np.array([[0, 1, 0, 0.0507],
                        [-1,0, 0, 0],
                        [0, 0, 1, -0.198], # robot_base
                        [0, 0, 0, 1]]).reshape(4,4)


        r1 =  np.array([[np.cos(theta[0]), - np.sin(theta[0]),0,0],
                        [np.sin(theta[0]), np.cos(theta[0]),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]).reshape(4,4)

        t1 = np.array([[1, 0, 0, a[0]],
                        [0, 1, 0, 0],
                        [0, 0, 1, -d[0]],
                        [0, 0, 0, 1]]).reshape(4,4)

        r2 = np.array([[1, 0, 0,  -a[1]*np.sin(theta[1])],
                        [0, 1, 0, 0],
                        [0, 0, 1, -a[1]*np.cos(theta[1])],
                        [0, 0, 0, 1]]).reshape(4,4)


        r3 = np.array([[1, 0, 0, a[2]*np.cos(theta[2])],
                        [0, 1, 0, 0],
                        [0, 0, 1, -a[2]*np.sin(theta[2])],
                        [0, 0, 0, 1]]).reshape(4,4)

        t4 = np.array([[1, 0, 0, d[3]],
                        [0, 1, 0, 0],
                        [0, 0, 1, a[3]],
                        [0, 0, 0, 1]]).reshape(4,4)

        r4 = np.array([[np.cos(theta[3]), - np.sin(theta[3]),0,0],
                        [np.sin(theta[3]), np.cos(theta[3]),0,0],
                        [0,0,1,0],
                        [0,0,0,1]]).reshape(4,4)

        T = tb @ r0 @ t0 @ r1 @ t1 @ r2 @ r3 @ t4 @ r4
        # self.linkT = [tb, r0 @ t0, tb @ t0 @ r1 @ t1, tb @ t0 @ r1 @ t1 @ r2,
        #             tb @ t0 @ r1 @ t1 @ r2 @ r3, tb @ t0 @ r1 @ t1 @ r2 @ r3 @ t4 @ r4]


        return T    
    
    def getLinkTransform(self, link):

        return self.linkT[link]

    
    def jacobian(self, link):


        # # Manipulator Jacobian in world frame
        q1_x = (self.a[0]*((-np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) + np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.cos(self.theta[0]) - (np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) + np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.sin(self.theta[0])) - self.a[1]*((-np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) + np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.cos(self.theta[0]) - (np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) + np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.sin(self.theta[0]))*np.sin(self.theta[1]) + self.a[2]*((-np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) + np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.cos(self.theta[0]) - (np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) + np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.sin(self.theta[0]))*np.cos(self.theta[2]) + self.d[3]*((-np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) + np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.cos(self.theta[0]) - (np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) + np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.sin(self.theta[0])))

        q1_y = (self.a[0]*(-(np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) - np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.sin(self.theta[0]) + (np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) + np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.cos(self.theta[0])) - self.a[1]*(-(np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) - np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.sin(self.theta[0]) + (np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) + np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.cos(self.theta[0]))*np.sin(self.theta[1]) + self.a[2]*(-(np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) - np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.sin(self.theta[0]) + (np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) + np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.cos(self.theta[0]))*np.cos(self.theta[2]) + self.d[3]*(-(np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) - np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.sin(self.theta[0]) + (np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) + np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.cos(self.theta[0])))
        
        q1_z = 0

        q2_x = (-self.a[1]*((-np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) + np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.sin(self.theta[0]) + (np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) + np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.cos(self.theta[0]))*np.cos(self.theta[1]))

        q2_y = (-self.a[1]*((np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) - np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.cos(self.theta[0]) + (np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) + np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.sin(self.theta[0]))*np.cos(self.theta[1]))
        
        q2_z =  (self.a[1]*np.sin(self.theta[1]))

        q3_x = (-self.a[2]*((-np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) + np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.sin(self.theta[0]) + (np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) + np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.cos(self.theta[0]))*np.sin(self.theta[2]))
        q3_y = (-self.a[2]*((np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) - np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.cos(self.theta[0]) + (np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) + np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.sin(self.theta[0]))*np.sin(self.theta[2]))
        q3_z = (-self.a[2]*np.cos(self.theta[2]))
        q4_x = 0
        q4_y = 0
        q4_z = 0
        q_r_x = (self.a[0]*((-np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) + np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.cos(self.theta[0]) + (-np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) - np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.sin(self.theta[0])) - self.a[1]*((-np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) + np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.cos(self.theta[0]) + (-np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) - np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.sin(self.theta[0]))*np.sin(self.theta[1]) + self.a[2]*((-np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) + np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.cos(self.theta[0]) + (-np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) - np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.sin(self.theta[0]))*np.cos(self.theta[2]) + self.d[3]*((-np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) + np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.cos(self.theta[0]) + (-np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) - np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.sin(self.theta[0])) - 0.0507*np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) - 0.0507*np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))
        q_r_y = (self.a[0]*((-np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) + np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.sin(self.theta[0]) + (np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) + np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.cos(self.theta[0])) - self.a[1]*((-np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) + np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.sin(self.theta[0]) + (np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) + np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.cos(self.theta[0]))*np.sin(self.theta[1]) + self.a[2]*((-np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) + np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.sin(self.theta[0]) + (np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) + np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.cos(self.theta[0]))*np.cos(self.theta[2]) + self.d[3]*((-np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) + np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))*np.sin(self.theta[0]) + (np.sin(self.robot_pose[2,0])*np.cos(self.robot_r) + np.sin(self.robot_r)*np.cos(self.robot_pose[2,0]))*np.cos(self.theta[0])) - 0.0507*np.sin(self.robot_pose[2,0])*np.sin(self.robot_r) + 0.0507*np.cos(self.robot_pose[2,0])*np.cos(self.robot_r))
        q_r_z = 0
        q_x_x = (np.cos(self.robot_pose[2,0]))
        q_x_y = (np.sin(self.robot_pose[2,0]))
        q_x_z = 0


        J = np.array([[q_r_x,q_x_x, q1_x,q2_x,q3_x,q4_x ],
                      [q_r_y,q_x_y, q1_y,q2_y,q3_y,q4_y ],
                      [q_r_z,q_x_z, q1_z,q2_z,q3_z,q4_z ],
                      [0, 0, 0   ,0   ,0   ,0 ],
                      [0, 0, 0   ,0   ,0   ,0 ],
                      [1, 0, 1   ,0   ,0   ,1 ]])
        

       
        

        return J

    
    def jacobianLink(self, T, revolute, link): 

        J = np.zeros((1, len(revolute)))    # Initialize J

        J[0, link] = 1
        
        return J

    def getEEJacobian(self):
        return self.jacobian(self.dof)
    
    def setRobotPose(self,value):
        self.robot_pose[0,0] = value[0,0]
        self.robot_pose[1,0] = value[1,0]
        self.robot_pose[2,0] = value[2,0]
    
    def getEETransform(self):
        return self.T
    
    def getJointPos(self,joint):
        # print(f'joint:{ self.q[joint]}')
        # print(f'q:{self.q}')
        # print(f'current joint:{self.q[joint]}')
        return self.q[joint]
    
    def getDOF(self):
        return self.dof