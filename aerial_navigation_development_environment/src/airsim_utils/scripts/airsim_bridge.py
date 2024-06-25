#!/usr/bin/env python3
import airsim
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

from std_msgs.msg import Float32

client = airsim.MultirotorClient()
client.enableApiControl(True)
client.armDisarm(True)
client.confirmConnection()

vehicle_odometry = Odometry()
client.simSetTraceLine([1.0, 1.0, 1.0, 1.0], 1) # enable trace line for vehicle 1 (the drone)

def odometry_callback(data):
    global vehicle_odometry
    vehicle_odometry = data
    vehicle_position = airsim.Vector3r(vehicle_odometry.pose.pose.position.x, -vehicle_odometry.pose.pose.position.y, -vehicle_odometry.pose.pose.position.z)
    vehicle_orientation = airsim.Quaternionr(vehicle_odometry.pose.pose.orientation.x, -vehicle_odometry.pose.pose.orientation.y, -vehicle_odometry.pose.pose.orientation.z, vehicle_odometry.pose.pose.orientation.w)
    client.simSetVehiclePose(airsim.Pose(vehicle_position, vehicle_orientation), True) # sets the position and orientation of the drone in the world  and enables collisions 

if __name__ == '__main__':
    rospy.init_node('airsim_bridge', anonymous=True)
    rospy.Subscriber("/state_estimation", Odometry, odometry_callback)
    rospy.spin()

    
