#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import rospy as rp
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray



def lidar_callback(data):

    # front = list(data.ranges[0:21])
    # right = list(data.ranges[0:41])
    # left = list(data.ranges[320:359])

    front = list(data.ranges[0:21])
    left = list(data.ranges[0:180])
    right = list(data.ranges[181:359])
    
    # recuperer les angles pour chaque raie 
    angle_min = data.angle_min 
    angle_incre = data.angle_increment

    ray_angle = []
    for i in range(len(data.ranges)):
        ray_angle.append(angle_min + (i*angle_incre))

    angle_right = ray_angle[181:359]
    angle_left = ray_angle[0:180]
    # preprocess the data 
    for i in range(len(front)):
        if str(front[i]) == "inf":
            front[i] = data.range_max
        front[i] = float(front[i])
    
    for i in range(len(left)):
        if str(left[i]) == "inf":
            left[i] = data.range_max
        left[i] = float(left[i])

    for i in range(len(right)):
        if str(right[i]) == "inf":
            right[i] = data.range_max
        right[i] = float(right[i])
    
    #print(f"Front : {front[0]}") 
    #print(f"Left : {left[35]}")
    #print(f"Right : {right[75]}")
    #print(f"Front : {np.min(front)}") 
    #print(f"Furthest Left point : {np.argmax(left)}")
    #print(f"Futhest Right point : {np.argmax(right)}")
    print(" ")
    print(f"Front : {front[0]}") 
    print(f"Left : {np.min(left)}")
    print(f"Right : {np.min(right)}")
    print("-----------------------")
        
    data_array = Float32MultiArray()

    data_array.data = [front,left,right]

    #pub_lidar.publish(data_array)


if __name__ == '__main__':


    rp.init_node('lidar_node',anonymous=True)
    rp.loginfo_once('Lidar node started')
    rp.Subscriber('/scan',LaserScan,callback=lidar_callback)
    
    #pub_lidar = rp.Publisher('/lidar_data',Float32MultiArray,queue_size=10)
    #pub = rp.Publisher('/cmd_vel',Twist,queue_size=10)
    try: 
        
        rp.spin()
    except KeyboardInterrupt:
        print('Close')
    
