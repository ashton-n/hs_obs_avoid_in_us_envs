#!/usr/bin/env python3
import airsim
import time
import sys
import rospy
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

from std_msgs.msg import Float32

client = airsim.MultirotorClient()
client.enableApiControl(True)
client.armDisarm(True)
client.confirmConnection()

vehicle_odometry = Odometry()

# Trees     Start:  -20600, -17550, 200
# Pillars   Start:  -20600, -5700,  200


# Starting and finsihsing points for each level
level_1_start  = 26.4 
level_1_finish = 126.4
level_1_start_flag = True     
level_1_finish_flag = True

level_2_start  = 127.4
level_2_finish = 227.4
level_2_start_flag = True
level_2_finish_flag = True

level_3_start = 228.4
level_3_finish = 328.4
level_3_start_flag = True
level_3_finish_flag = True

level_4_start = 329.4
level_4_finish = 429.4
level_4_start_flag = True
level_4_finish_flag = True

level_5_start = 430.4
level_5_finish = 530.4
level_5_start_flag = True
level_5_finish_flag = True

level_6_start = 531.4
level_6_finish = 631.4
level_6_start_flag = True
level_6_finish_flag = True

level_7_start = 632.4
level_7_finish = 732.4
level_7_start_flag = True
level_7_finish_flag = True

level_8_start = 733.4
level_8_finish = 833.4
level_8_start_flag = True
level_8_finish_flag = True

level_9_start = 834.4
level_9_finish = 934.4
level_9_start_flag = True
level_9_finish_flag = True

level_10_start = 935.4
level_10_finish = 1035.4
level_10_start_flag = True
level_10_finish_flag = True


collision_count = 0
save_odom_count = 0
save_odom_interval = 20
vehicle_position = airsim.Vector3r(0, 0, 0)
vehicle_orientation = airsim.Quaternionr(0, 0, 0)
altitude = []
speed = []
speed_flag = True
start_time = time.time()

dist_point_1 = np.array([0, 0, 0, np.uint64(0)])
dist_point_2 = np.array([0, 0, 0, np.uint64(0)])

time_scale = 1

def get_inst_speed(state):
    # calculate the instantaneous speed of the MAV
    global dist_point_1
    global dist_point_2
    global time_scale

    if dist_point_1[3] == 0:
        dist_point_1[3] = state.timestamp
        dist_point_2[3] = state.timestamp
        dist_point_1[:3] = np.array([state.kinematics_estimated.position.x_val, state.kinematics_estimated.position.y_val, state.kinematics_estimated.position.z_val])
        dist_point_2[:3] = np.array([state.kinematics_estimated.position.x_val, state.kinematics_estimated.position.y_val, state.kinematics_estimated.position.z_val])
        return 0

    time_now = state.timestamp
    x, y, z = state.kinematics_estimated.position.x_val, state.kinematics_estimated.position.y_val, state.kinematics_estimated.position.z_val
    dist_point_1 = dist_point_2
    dist_point_2 = np.array([x, y, z, time_now])
    dist = np.linalg.norm(dist_point_2[:3] - dist_point_1[:3])
    time_diff = (dist_point_2[3] - dist_point_1[3]) * 1e-9 
    speed = dist / time_diff

    return speed 

def get_avg_speed(speed_list):
    # calculate and return the average 
    return np.average(speed_list)

def divide_into_three_and_avg(altitude_list):
    # Calculate the length of each part
    length = len(altitude_list)
    part_length = length // 3

    # Use list slicing to divide the list into three parts
    beginning = altitude_list[:part_length]
    middle = altitude_list[part_length:2 * part_length]
    ending = altitude_list[2 * part_length:]

    avg_1 = np.average(beginning)
    avg_2 = np.average(middle)
    avg_3 = np.average(ending)

    return [avg_1, avg_2, avg_3]

# create an interrupt handler that triggers every second and stores the drone's position and orientation
def save_odom(x, y, z, roll, pitch, yaw, w):
    global vehicle_position
    global vehicle_orientation
    global save_odom_count
    save_odom_count = 0
    vehicle_position = airsim.Vector3r(x, y, z)
    vehicle_orientation = airsim.Quaternionr(roll, pitch, yaw, w)

def odometry_callback(data):
    # callback function to update MAVs state, check for collisions and log metrics
    global client
    collision_info = client.simGetCollisionInfo()
    state = client.getMultirotorState()
    
    global start_time
    global vehicle_odometry
    
    vehicle_odometry = data
    x, y ,z = vehicle_odometry.pose.pose.position.x, -vehicle_odometry.pose.pose.position.y, -vehicle_odometry.pose.pose.position.z
    roll, pitch, yaw, w = vehicle_odometry.pose.pose.orientation.x, -vehicle_odometry.pose.pose.orientation.y, -vehicle_odometry.pose.pose.orientation.z, vehicle_odometry.pose.pose.orientation.w

    global level_1_start
    global level_1_finish
    global level_1_start_flag
    global level_1_finish_flag

    global level_2_start
    global level_2_finish
    global level_2_start_flag
    global level_2_finish_flag

    global level_3_start
    global level_3_finish
    global level_3_start_flag
    global level_3_finish_flag

    global level_4_start
    global level_4_finish
    global level_4_start_flag
    global level_4_finish_flag

    global level_5_start
    global level_5_finish
    global level_5_start_flag
    global level_5_finish_flag

    global level_6_start
    global level_6_finish
    global level_6_start_flag
    global level_6_finish_flag

    global level_7_start
    global level_7_finish
    global level_7_start_flag
    global level_7_finish_flag

    global level_8_start
    global level_8_finish
    global level_8_start_flag
    global level_8_finish_flag

    global level_9_start
    global level_9_finish
    global level_9_start_flag
    global level_9_finish_flag

    global level_10_start
    global level_10_finish
    global level_10_start_flag
    global level_10_finish_flag

    global collision_count
    global save_odom_count
    global save_odom_interval
    global vehicle_position
    global vehicle_orientation
    global altitude
    global speed
    global speed_flag
    

    # Level 1
    if x > level_1_start and x < level_1_finish:
        if level_1_start_flag:
            level_1_start_flag = False
            start_time = time.time()
            speed.clear()
        if save_odom_count == save_odom_interval:
            save_odom(x, y, z, roll, pitch, yaw, w)
            altitude.append(z)
            save_odom_count = 0

            if speed_flag:
                get_inst_speed(state)
                speed_flag = False
            else:
                speed.append(get_inst_speed(state))
        
        save_odom_count += 1

        if collision_info.has_collided or state.collision.has_collided:
            # if a coillision has occured, increment the collision count
            print("LEVEL 1 COLLISION DETECTED!")
            collision_count += 1
            return

    if x > level_1_finish and x < level_2_start and level_1_finish_flag:
        
        finish_time = (time.time() - start_time) * time_scale   
        start_time = time.time()
        level_1_finish_flag = False
        alts = divide_into_three_and_avg(altitude)
        print("Level 1 Time: {:.3f} Average Speed: {:.3f} Start Alt.: {:.3f} Mid Alt.: {:.3f} End Alt.: {:.3f} Collision Count: {}".format(finish_time, get_avg_speed(speed), alts[0], alts[1], alts[2], collision_count))
        altitude.clear()
        speed.clear()
        collision_count = 0
    

    
    # Level 2
    if x > level_2_start and x < level_2_finish:
        if level_2_start_flag:
            level_2_start_flag = False
            start_time = time.time()
            speed.clear()
        if save_odom_count == save_odom_interval:
            save_odom(x, y, z, roll, pitch, yaw, w)
            altitude.append(z)
            save_odom_count = 0
            speed.append(get_inst_speed(state))
        save_odom_count += 1
        if collision_info.has_collided or state.collision.has_collided:
            print("LEVEL 2 COLLISION DETECTED!")
            collision_count += 1
            return
    if x > level_2_finish and x < level_3_start and level_2_finish_flag:
        finish_time = (time.time() - start_time) * time_scale 
        start_time = time.time()
        level_2_finish_flag = False
        alts = divide_into_three_and_avg(altitude)
        print("Level 2 Time: {:.3f} Average Speed: {:.3f} Start Alt.: {:.3f} Mid Alt.: {:.3f} End Alt.: {:.3f} Collision Count: {}".format(finish_time, get_avg_speed(speed), alts[0], alts[1], alts[2], collision_count))
        altitude.clear()
        collision_count = 0
    
    # Level 3
    if x > level_3_start and x < level_3_finish:
        if level_3_start_flag:
            level_3_start_flag = False
            start_time = time.time()
            speed.clear()
        if save_odom_count == save_odom_interval:
            save_odom(x, y, z, roll, pitch, yaw, w)
            altitude.append(z)
            save_odom_count = 0
            speed.append(get_inst_speed(state))
        save_odom_count += 1
        if collision_info.has_collided or state.collision.has_collided:
            print("LEVEL 3 COLLISION DETECTED!")
            collision_count += 1
            return
    if x > level_3_finish and x < level_4_start and level_3_finish_flag:
        finish_time = (time.time() - start_time) * time_scale 
        start_time = time.time()
        level_3_finish_flag = False
        alts = divide_into_three_and_avg(altitude)
        print("Level 3 Time: {:.3f} Average Speed: {:.3f} Start Alt.: {:.3f} Mid Alt.: {:.3f} End Alt.: {:.3f} Collision Count: {}".format(finish_time, get_avg_speed(speed), alts[0], alts[1], alts[2], collision_count))
        altitude.clear()
        collision_count = 0
    
    # Level 4
    if x > level_4_start and x < level_4_finish:
        if level_4_start_flag:
            level_4_start_flag = False
            start_time = time.time()
            speed.clear()
        if save_odom_count == save_odom_interval:
            save_odom(x, y, z, roll, pitch, yaw, w)
            altitude.append(z)
            save_odom_count = 0
            speed.append(get_inst_speed(state))
        save_odom_count += 1
        if collision_info.has_collided or state.collision.has_collided:
            print("LEVEL 4 COLLISION DETECTED!")
            collision_count += 1
            return
    if x > level_4_finish and x < level_5_start and level_4_finish_flag:
        finish_time = (time.time() - start_time) * time_scale 
        start_time = time.time()
        level_4_finish_flag = False
        alts = divide_into_three_and_avg(altitude)
        print("Level 4 Time: {:.3f} Average Speed: {:.3f} Start Alt.: {:.3f} Mid Alt.: {:.3f} End Alt.: {:.3f} Collision Count: {}".format(finish_time, get_avg_speed(speed), alts[0], alts[1], alts[2], collision_count)) 
        altitude.clear()
        collision_count = 0

    # Level 5
    if x > level_5_start and x < level_5_finish:
        if level_5_start_flag:
            level_5_start_flag = False
            start_time = time.time()
            speed.clear()
        if save_odom_count == save_odom_interval:
            save_odom(x, y, z, roll, pitch, yaw, w)
            altitude.append(z)
            save_odom_count = 0
            speed.append(get_inst_speed(state))
        save_odom_count += 1
        if collision_info.has_collided or state.collision.has_collided:
            print("LEVEL 5 COLLISION DETECTED!")
            collision_count += 1
            return
    if x > level_5_finish and x < level_6_start and level_5_finish_flag:
        finish_time = (time.time() - start_time) * time_scale 
        start_time = time.time()
        level_5_finish_flag = False
        alts = divide_into_three_and_avg(altitude)
        print("Level 5 Time: {:.3f} Average Speed: {:.3f} Start Alt.: {:.3f} Mid Alt.: {:.3f} End Alt.: {:.3f} Collision Count: {}".format(finish_time, get_avg_speed(speed), alts[0], alts[1], alts[2], collision_count))
        altitude.clear()
        collision_count = 0

    # Level 6
    if x > level_6_start and x < level_6_finish:
        if level_6_start_flag:
            level_6_start_flag = False
            start_time = time.time()
            speed.clear()
        if save_odom_count == save_odom_interval:
            save_odom(x, y, z, roll, pitch, yaw, w)
            altitude.append(z)
            save_odom_count = 0
            speed.append(get_inst_speed(state))
        save_odom_count += 1
        if collision_info.has_collided or state.collision.has_collided:
            print("LEVEL 6 COLLISION DETECTED!")
            collision_count += 1
            return
    if x > level_6_finish and x < level_7_start and level_6_finish_flag:
        finish_time = (time.time() - start_time) * time_scale 
        start_time = time.time()
        level_6_finish_flag = False
        alts = divide_into_three_and_avg(altitude)
        print("Level 6 Time: {:.3f} Average Speed: {:.3f} Start Alt.: {:.3f} Mid Alt.: {:.3f} End Alt.: {:.3f} Collision Count: {}".format(finish_time, get_avg_speed(speed), alts[0], alts[1], alts[2], collision_count))
        altitude.clear()
        collision_count = 0
    
    # Level 7
    if x > level_7_start and x < level_7_finish:
        if level_7_start_flag:
            level_7_start_flag = False
            start_time = time.time()
            speed.clear()
        if save_odom_count == save_odom_interval:
            save_odom(x, y, z, roll, pitch, yaw, w)
            altitude.append(z)
            save_odom_count = 0
            speed.append(get_inst_speed(state))
        save_odom_count += 1
        if collision_info.has_collided or state.collision.has_collided:
            print("LEVEL 7 COLLISION DETECTED!")
            collision_count += 1
            return
    if x > level_7_finish and x < level_8_start and level_7_finish_flag:
        finish_time = (time.time() - start_time) * time_scale 
        start_time = time.time()
        level_7_finish_flag = False
        alts = divide_into_three_and_avg(altitude)
        print("Level 7 Time: {:.3f} Average Speed: {:.3f} Start Alt.: {:.3f} Mid Alt.: {:.3f} End Alt.: {:.3f} Collision Count: {}".format(finish_time, get_avg_speed(speed), alts[0], alts[1], alts[2], collision_count))
        altitude.clear()
        collision_count = 0
    
    # Level 8
    if x > level_8_start and x < level_8_finish:
        if level_8_start_flag:
            level_8_start_flag = False
            start_time = time.time()
            speed.clear()
        if save_odom_count == save_odom_interval:
            save_odom(x, y, z, roll, pitch, yaw, w)
            altitude.append(z)
            save_odom_count = 0
            speed.append(get_inst_speed(state))
        save_odom_count += 1
        if collision_info.has_collided or state.collision.has_collided:
            print("LEVEL 8 COLLISION DETECTED!")
            collision_count += 1
            return
    if x > level_8_finish and x < level_9_start and level_8_finish_flag:
        finish_time = (time.time() - start_time) * time_scale 
        start_time = time.time()
        level_8_finish_flag = False
        alts = divide_into_three_and_avg(altitude)
        print("Level 8 Time: {:.3f} Average Speed: {:.3f} Start Alt.: {:.3f} Mid Alt.: {:.3f} End Alt.: {:.3f} Collision Count: {}".format(finish_time, get_avg_speed(speed), alts[0], alts[1], alts[2], collision_count))
        altitude.clear()
        collision_count = 0

    # Level 9
    if x > level_9_start and x < level_9_finish:
        if level_9_start_flag:
            level_9_start_flag = False
            start_time = time.time()
            speed.clear()
        if save_odom_count == save_odom_interval:
            save_odom(x, y, z, roll, pitch, yaw, w)
            altitude.append(z)
            save_odom_count = 0
            speed.append(get_inst_speed(state))
        save_odom_count += 1
        if collision_info.has_collided or state.collision.has_collided:
            print("LEVEL 9 COLLISION DETECTED!")
            collision_count += 1
            return
    if x > level_9_finish and x < level_10_start and level_9_finish_flag:
        finish_time = (time.time() - start_time) * time_scale 
        start_time = time.time()
        level_9_finish_flag = False
        alts = divide_into_three_and_avg(altitude)
        print("Level 9 Time: {:.3f} Average Speed: {:.3f} Start Alt.: {:.3f} Mid Alt.: {:.3f} End Alt.: {:.3f} Collision Count: {}".format(finish_time, get_avg_speed(speed), alts[0], alts[1], alts[2], collision_count))
        altitude.clear()
        collision_count = 0

    # Level 10
    if x > level_10_start and x < level_10_finish:
        if level_10_start_flag:
            level_10_start_flag = False
            start_time = time.time()
            speed.clear()
        if save_odom_count == save_odom_interval:
            save_odom(x, y, z, roll, pitch, yaw, w)
            altitude.append(z)
            save_odom_count = 0
            speed.append(get_inst_speed(state))
        save_odom_count += 1
        if collision_info.has_collided or state.collision.has_collided:
            print("LEVEL 10 COLLISION DETECTED!")
            collision_count += 1
            return
    if x > level_10_finish and level_10_finish_flag:
        finish_time = (time.time() - start_time) * time_scale 
        start_time = time.time()
        level_10_finish_flag = False
        alts = divide_into_three_and_avg(altitude)
        print("Level 10 Time: {:.3f} Average Speed: {:.3f} Start Alt.: {:.3f} Mid Alt.: {:.3f} End Alt.: {:.3f} Collision Count: {}".format(finish_time, get_avg_speed(speed), alts[0], alts[1], alts[2], collision_count))
        altitude.clear()
        collision_count = 0
        print("Experiment Complete")
        client.armDisarm(False)
        client.enableApiControl(False)
        sys.exit(0)


if __name__ == '__main__':
    rospy.init_node('log_metrics', anonymous=True)
    rospy.Subscriber("/state_estimation", Odometry, odometry_callback)
    rospy.spin()

    
