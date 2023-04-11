#!/usr/bin/env python 
import rospy 
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
import numpy as np

#This class will do the following:
#   subscribe to the /cmd_vel topic 
#   publish the simulated pose of the robot to /pose_sim topic 
#   publish to /wr and /wl the simulated wheel speed. 
class PuzzlebotKinClass(): 
    def __init__(self): 
        ###******* INIT PUBLISHERS *******### 
        # Create the subscriber to cmd_vel topic
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb)
        # Create ROS publishers
        self.pose_sim_pub = rospy.Publisher('pose_sim', PoseStamped ,queue_size=1) #Publisher to pose_sim topic
        self.wr_pub = rospy.Publisher('wr', Float32 ,queue_size=1) # Publisher to wr topic
        self.wl_pub = rospy.Publisher('wl', Float32 ,queue_size=1) # Publisher to wl topic
        self.marker_pub = rospy.Publisher("/robot_marker", Marker, queue_size = 1)

        ############ ROBOT CONSTANTS ################ 
        self.r=0.065 #puzzlebot wheel radius [m]
        self.L = 0.19 #puzzlebot wheel separation [m]
        self.delta_t = 0.02 # Desired time to update the robot's pose [s]
        ############ Variables ###############
        self.w = 0.0 # robot's angular speed [rad/s]
        self.v = 0.0 #robot's linear speed [m/s]
        self.index = 0.0
        self.y_act = 0.0
        self.x_act = 0.0
        self.theta = 0.0 # robot's angular speed [rad/s]

        self.pose_stamped = PoseStamped()
        ############ Flags  #################
        self.lock_cmd_vel = 0 # This flag will be used to avoid updating the robot's speed when we are computing the robot's pose. 
        
        rate = rospy.Rate(int(1.0/self.delta_t)) # The rate of the while loop will be the inverse of the desired delta_t
        while not rospy.is_shutdown():
            ####### Add / modify  your code here #############
            [x, y, theta] = self.get_robot_pose(self.v, self.w)
            pose_stamped = self.get_pose_stamped(x, y, theta)
            [wl, wr] = self.get_wheel_speeds()

            ######## Publish a marker to rviz #########
            marker = self.fill_marker(pose_stamped) # marcador lleno
            self.marker_pub.publish(marker) # se publica

            ######## Publish the data #################
            self.pose_sim_pub.publish(pose_stamped)
            self.wr_pub.publish(wr)
            self.wl_pub.publish(wl)
            rate.sleep()
    
    def cmd_vel_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
    
    def get_wheel_speeds(self):
        rospy.logwarn("get_wheel_speeds, according to formula:")
        wl = (2*self.v - (self.w*self.L))/(2*self.r) # Left wheel angular speed in [rad/s]
        wr = (2*self.v + (self.w*self.L))/(2*self.r) # Right wheel angular speed in [rad/s]
        return [wl, wr]
   
    def get_pose_stamped(self, x, y, yaw):
        # x, y and yaw are the robot's position (x,y) and orientation (yaw)
        # Write the data as a ROS PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "odom"    #This can be changed in this case I'm using a frame called odom.
        pose_stamped.header.stamp = rospy.Time.now()
        # Position
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        # Rotation of the mobile base frame w.r.t. "map" frame as a quaternion
        quat = quaternion_from_euler(0,0,yaw)
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]
        return pose_stamped
    
    def get_robot_pose(self, v, w):
        #This functions receives the robot speed v [m/s] and w [rad/s]
        # and gets the robot's pose (x, y, theta) where (x,y) are in [m] and (theta) in [rad]
        # is the orientation,     
        ############ MODIFY THIS CODE   ################
        rospy.logwarn("set_robot_pose: Make sure to modify this function")
        self.theta = self.theta + w*self.delta_t
        self.y_act = self.y_act + v*self.delta_t*np.sin(self.theta)
        self.x_act = self.x_act + v*self.delta_t*np.cos(self.theta)
        return [self.x_act, self.y_act, self.theta]

    def fill_marker(self, pose_stamped=PoseStamped()):
        # This function will fill the necessary data tu publish a marker to rviz. 
        # It receives pose_stamped which must be a [geometry_msgs/PoseStamped] message. 
        marker = Marker()
        marker.header.frame_id = pose_stamped.header.frame_id
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3; mesh_resource: 10
        marker.mesh_resource = "package://puzzlebot_sim/meshes/bases/MCR2_1000_0_Puzzlebot.stl"
        marker.mesh_use_embedded_materials = True
        marker.type = 10
        marker.id = 0
        

        # Set the scale of the marker
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        # Set the color
        marker.color.r = 1.0 #values between 0 and 1
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = pose_stamped.pose.position.x
        marker.pose.position.y = pose_stamped.pose.position.y
        marker.pose.position.z = 0
        marker.pose.orientation.x = pose_stamped.pose.orientation.x
        marker.pose.orientation.y = pose_stamped.pose.orientation.y
        marker.pose.orientation.z = pose_stamped.pose.orientation.z
        marker.pose.orientation.w = pose_stamped.pose.orientation.w
        return marker

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    # first thing, init a node!
    rospy.init_node('puzzlebot_kinematic_model') 
    PuzzlebotKinClass()