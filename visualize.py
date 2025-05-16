#!/usr/bin/python

import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped,Point, Pose
import rospy
from nav_msgs.msg  import Path
from tf.transformations import quaternion_from_euler
from std_msgs.msg import ColorRGBA

def publish_current_pose(transformation, publisher,frame_id ="world_ned"):
        

        position = transformation[0:3,3]
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp=rospy.Time.now()
  
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        roatation = transformation[0:3,0:3]

        theta = np.arctan2(roatation[1,0],roatation[0,0])
        # print(f'end-effector:{position[0]:.3f} {position[1]:.3f} {theta:.3f}')
        q = quaternion_from_euler(0, 0, theta)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        
        publisher.publish(pose_msg)

def publish_robot_pose(pose, publisher,frame_id ="world_ned"):
        

        
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp=rospy.Time.now()
  
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        pose_msg.pose.position.z = 0

        q = quaternion_from_euler(0, 0, pose[2]) 
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        
        publisher.publish(pose_msg)

def publish_goal_pose(position, publisher,frame_id ="world_ned",current = None):
        


        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp=rospy.Time.now()
        if len(position) == 3:
                pose_msg.pose.position.x = position[0]
                pose_msg.pose.position.y = position[1]
                pose_msg.pose.position.z = position[2]
                pose_msg.pose.orientation.x = 0
                pose_msg.pose.orientation.y = 0
                pose_msg.pose.orientation.z = 0
                pose_msg.pose.orientation.w = 1
        elif len(position) == 4:
                pose_msg.pose.position.x = position[0]
                pose_msg.pose.position.y = position[1]
                pose_msg.pose.position.z = position[2]
                q = quaternion_from_euler(0, 0, position[3]) 
                pose_msg.pose.orientation.x = q[0]
                pose_msg.pose.orientation.y = q[1]
                pose_msg.pose.orientation.z = q[2]
                pose_msg.pose.orientation.w = q[3]
        elif len(position) == 1:
                position = current
                pose_msg.pose.position.x = position[0]
                pose_msg.pose.position.y = position[1]
                pose_msg.pose.position.z = position[2]
                q = quaternion_from_euler(0, 0, position[0]) 
                pose_msg.pose.orientation.x = q[0]
                pose_msg.pose.orientation.y = q[1]
                pose_msg.pose.orientation.z = q[2]
                pose_msg.pose.orientation.w = q[3]

        
        publisher.publish(pose_msg)


def publish_goal_marker(position,publisher,frame_id ="world_ned"):
        marker = Marker()
        marker.header.frame_id = frame_id  # Or whatever your robot's base frame is
        marker.header.stamp = rospy.Time.now()

        marker.ns = "goal_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position (desired position)
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]

        # Orientation (no rotation)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Size of the sphere
        marker.scale.x = 0.02  # 2cm diameter
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        

        # Color (e.g., red)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # Alpha = 1.0 means fully opaque

        publisher.publish(marker)



def publish_trajectory_marker(trajectory,limits,publisher,frame_id ="world_ned"):
        marker = Marker()
        marker.header.frame_id = frame_id 
        marker.header.stamp = rospy.Time.now()

        marker.ns = "trajectory_marker"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Line strip needs a list of points
        marker.points = []
        marker.colors = []

        for position, limit in zip(trajectory,limits):
                # Add current position
                p_start = Point()
                p_start.x = position[0]
                p_start.y = position[1]
                p_start.z = position[2]

                marker.points.append(p_start)

                color = ColorRGBA()
                if limit:
                        # If limit was reached, RED
                        color.r = 1.0
                        color.g = 0.0
                        color.b = 0.0
                        color.a = 1.0

                else:
                        # Normal movement, GREEN
                        color.r = 0.0
                        color.g = 1.0
                        color.b = 0.0
                        color.a = 1.0

                marker.colors.append(color)


        # Line width
        marker.scale.x = 0.005  # 5 mm thick line


        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        publisher.publish(marker)