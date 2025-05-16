#!/usr/bin/python

import cv2
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image

from cv_bridge import CvBridge , CvBridgeError
from std_msgs.msg import Bool
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix

from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray



class aruco_detection:
    def __init__(self):

       
        #subscribers
        self.odom_sub = rospy.Subscriber("/turtlebot/kobuki/odom_ground_truth", Odometry, self.get_odom)
        self.camImage_sub = rospy.Subscriber("/turtlebot/kobuki/realsense/color/image_color", Image, self.get_img)
        self.camInfo_sub = rospy.Subscriber("/turtlebot/kobuki/realsense/color/camera_info", CameraInfo, self.camera_info)

        #publishers  
        self.aruco_pose_pub = rospy.Publisher("/aruco_pose",Float64MultiArray, queue_size=10)
        self.aruco_marker = rospy.Publisher("/turtlebot/Aruco", MarkerArray, queue_size=10)
        self.aruco_img_pub = rospy.Publisher("/aruco/detected_image", Image, queue_size=10)
        self.arudo_tf_broadcaster = tf.TransformBroadcaster()

        self.bridge = CvBridge()

        self.robot_pose = None
        self.aruco_info = []
        self.id_aruco = []

    
    
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x,
                                                            odom.pose.pose.orientation.y,
                                                            odom.pose.pose.orientation.z,
                                                            odom.pose.pose.orientation.w])
        # self.robot_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, yaw])
        self.robot_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, 0, yaw])
        # print(f'robot pose:{self.robot_pose[0]:.3f}, {self.robot_pose[1]:.3f}, {self.robot_pose[2]:.3f}, {self.robot_pose[3]:.3f}')


    def Publish_Marker(self):                                                       
        marks = MarkerArray()
        for i in range(0,len(self.id_aruco)):
            marker = Marker()
            marker.header.frame_id = "world_ned" 
            marker.header.stamp = rospy.Time.now()
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            #marker pose wrt world frame
            marker.pose.position.x = self.aruco_info[i][0]
            marker.pose.position.y = self.aruco_info[i][1]
            marker.pose.position.z = self.aruco_info[i][2]

            # print(f'maker pos:{self.aruco_info[i][0]:.3f}, {self.aruco_info[i][1]:.3f}, {self.aruco_info[i][2]:.3f}')
            
            marker.scale.x = 0.1 
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            #the marker color
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.5
            
            marks.markers.append(marker)
        self.aruco_marker.publish(marks)

    def camera_info(self, msg):
        
        self.dist_coefficients = np.array(msg.D)
        self.camera_matrix = np.array(msg.K).reshape((3,3))

    def get_img(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8") 
        except CvBridgeError as e:
            print(e)
        
        

        marker_size = 0.05

        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL) 
        aruco_params = cv2.aruco.DetectorParameters_create()
        corners, detected_ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=aruco_params) 
        if len(corners)<1:
            print(f'Aruco Not Detected')
        
        # else:
            # print(f'detected {len(corners)} arucos')
        if detected_ids is not None:
            
            cv2.aruco.drawDetectedMarkers(image, corners, detected_ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, self.camera_matrix, self.dist_coefficients)
        

        if len(corners) > 0 and self.robot_pose is not None:
            
            for i in range(len(detected_ids)):
                cv2.aruco.drawAxis(image, self.camera_matrix,
                                   self.dist_coefficients, rvecs[i], tvecs[i], marker_size)
                
                #transformation of the camera wrt to robot frame 
                R = quaternion_matrix([0.500, 0.500, 0.500, 0.500])
                robot_T_camera = np.array([[0, 0, 1,  0.136],
                                    [1, 0, 0, -0.033],
                                    [0, 1, 0, -0.116],
                                    [0, 0, 0,    1]])
                
                robot_T_camera[0:3,0:3] = R[0:3,0:3]

                rvec = rvecs[i]  # Selecting the first marker
                tvec = tvecs[i]  # Selecting the first marker

                # Convert rotation vector to rotation matrix
                R, _ = cv2.Rodrigues(rvec)

                # Create a 4x4 transformation matrix
                camera_T_aruco = np.eye(4)
                camera_T_aruco[:3, :3] = R
                camera_T_aruco[:3, 3] = tvec.flatten()

                robot_T_aruco = robot_T_camera @ camera_T_aruco

                r_ArucoPose = np.array([[float(robot_T_aruco[0][-1])],
                                    [float(robot_T_aruco[1][-1])],
                                    [robot_T_aruco[2][-1]],
                                    [robot_T_aruco[3][-1]]])
                
                world_T_robot = np.array([[np.cos(self.robot_pose[3]),-np.sin(self.robot_pose[3]), 0, float(self.robot_pose[0])],
                                  [np.sin(self.robot_pose[3]), np.cos(self.robot_pose[3]), 0, float(self.robot_pose[1])],
                                  [0                 ,0                  , 1             , float(self.robot_pose[2])],
                                  [0                 ,0                  , 0             , 1]])

                #transformation of the Aruco wrt to world frame
                world_T_aruco = world_T_robot @ r_ArucoPose

                if detected_ids[i] not in self.id_aruco:
                    self.id_aruco.append(detected_ids[i])
                    self.aruco_info.append([world_T_aruco[0][-1],world_T_aruco[1][-1],world_T_aruco[2][-1]])

                print(f'aruco position:{world_T_aruco[0][0]:.3f}, {world_T_aruco[1][0]:.3f}, {world_T_aruco[2][0]:.3f}')
                # Broadcast TF frame for each detected ArUco marker
                marker_id = detected_ids[i][0]  # int
                marker_frame = f"aruco_{marker_id}"

                translation = (world_T_aruco[0][0], world_T_aruco[1][0], world_T_aruco[2][0])
                rotation_matrix = world_T_robot[:3, :3] @ robot_T_aruco[:3, :3]  # rotation in world frame
                quat = tf.transformations.quaternion_from_matrix(np.vstack((np.hstack((rotation_matrix, np.array([[0], [0], [0]]))), [0, 0, 0, 1])))

                self.arudo_tf_broadcaster.sendTransform(
                    translation,
                    quat,
                    rospy.Time.now(),
                    marker_frame,
                    "world_ned"
                )

                
            #Publish Aruco pose
            self.aruco_pose_pub.publish(Float64MultiArray(data=self.aruco_info))

            # resize image and show the output image
            width = int(image.shape[1] * 50 / 100)
            height = int(image.shape[0] * 50 / 100)
            img_size = (width, height)
            re_size = cv2.resize(image, img_size, interpolation = cv2.INTER_AREA)
            # cv2.imshow("Image", re_size)
            # cv2.waitKey(2)
            try:
                ros_image = self.bridge.cv2_to_imgmsg(re_size, "bgr8")
                ros_image.header.stamp = rospy.Time.now()
                self.aruco_img_pub.publish(ros_image)
            except CvBridgeError as e:
                print(e)

        self.Publish_Marker()
        return self.aruco_info
    




if __name__ == "__main__":
    
    rospy.init_node('aruco_detection')
    node = aruco_detection()
    rospy.spin()