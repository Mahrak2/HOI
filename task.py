#!/usr/bin/python

import numpy as np

import math



class Task():

    def __init__(self, name,desired):
        self.name = name
        self.sigma_d = desired
        self.feedfroward_velocity = None
        self.K = None
        self.active = True

    def update(self,robot):
        pass

    def setDesired(self,value):
        self.sigma_d = value

    def getDesired(self):
        return self.sigma_d
    
    def getJacobian(self):
        return self.J
    
    def getError(self):
        return self.err
    
    def set_feedfoward_velocity(self,value):
        self.feedfroward_velocity = value
    
    def get_feedforward_velocity(self):
        return self.feedfroward_velocity
    
    def set_K_gain(self,value):
        self.K = value
    
    def get_K_gain(self):
        return self.K
    
    def dist_to_obs(self):
        return self.distance_to_obstacle
    
    def is_Active(self):
        return self.active
    
    def get_activate_value(self):
        return self.activate_value

class Position3D(Task):
    def __init__ (self,name,desired,link):
        super().__init__(name,desired)
        self.link = link
        self.J = np.zeros((6,4))
        self.feedfroward_velocity = np.zeros((6,1))
        self.err = np.zeros(3)
        self.k = np.eye(3)
        self.active = True
        self.activate_value = 1
    
    def update(self,robot):

        self.J = robot.getEEJacobian()[0:3,:]
        self.err = (self.getDesired().reshape(3,1)-robot.getEETransform()[0:3,3].reshape(3,1)).reshape(3,1)
        # print(robot.getEETransform()[0:3,3])


class Orientation3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.J = np.zeros((1,4))# Initialize with proper dimensions
        self.err =np.zeros(1) # Initialize with proper dimensions
        self.link = link #Selected link
        self.feedforward_velocity = np.zeros((1,1)) #Initialize for feed-forward velocity
        self.K = np.eye(1)#Initialize for K gain 
        self.active = True
        self.activate_value = 1

    def update(self, robot):
        self.J = robot.getEEJacobian()[5,:].reshape(1,robot.getDOF()) # Update task Jacobian
        self.err = (self.getDesired() - math.atan2(robot.getEETransform()[1,0],\
                                                   robot.getEETransform()[0,0])).reshape(1,1)  # Update task error

class Configuration3D(Task):
    def __init__(self, name, desired, link):
        super().__init__(name, desired)
        self.J =np.zeros((4,4)) # Initialize with proper dimensions
        self.err = np.zeros(4) # Initialize with proper dimensions
        self.link = link #Selected link
        self.feedforward_velocity = np.zeros((4,1))#Initialize for feed-forward velocity
        self.K = np.eye(4)#Initialize for K gain 
        self.active = True
        self.activate_value = 1

    def update(self, robot):
        self.J = robot.getEEJacobian()[[0,1,2,5],:]
        sigma = np.zeros((4,1))
        sigma[0:3] = robot.getEETransform()[0:3,3].reshape(3,1) #Get current position
        sigma[3] = math.atan2(robot.getEETransform()[1,0], robot.getEETransform()[0,0])#Get current orientation
        self.err = (self.getDesired().reshape(4,1) - sigma).reshape(4,1)



class JointLimit(Task):
    def __init__(self, name, joint_set, threshold, link):
        super().__init__(name,joint_set)
        self.J = np.zeros(4)
        self.err = 0
        self.q_min = joint_set[0]
        self.q_max = joint_set[1]
        self.link = link
        self.threshold_a = threshold[0]
        self.threshold_d = threshold[1]
        self.activate_value = 0
        self.last_activate_value = 0

    def update(self,robot):
        # self.J = robot.getLinkJacobian(self.link)[5,:].reshape(1,robot.getDOF())
        self.J = np.zeros((1,robot.getDOF())) # Initialize task Jacobian
        self.J[0,self.link+1] = 1 # Update task Jacobian
        # print(f'J: {self.J}')
        q = robot.getJointPos(self.link+1)
                       
        if q >= (self.q_max - self.threshold_a):
            self.activate_value = -1      
        elif q <= (self.q_min + self.threshold_a):
            self.activate_value = 1   
        # Deactivate if within bounds
        if (self.activate_value == -1) and (q <= (self.q_max - self.threshold_d)):
            self.activate_value = 0
        elif (self.activate_value == 1) and (q >= (self.q_min + self.threshold_d)):
            self.activate_value = 0

        self.err = self.activate_value

        if self.activate_value != self.last_activate_value and self.activate_value != 0:
            print(f"Joint Limit Activated: {self.name} (Link {self.link}) - Activation Value: {self.activate_value}")
        self.last_activate_value = self.activate_value
