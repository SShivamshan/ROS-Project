#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy as rp
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
def lidar_callback(data):
    global front_data
    global left_data
    global right_data
    
    # pour le tunnel les valeurs doit etre comprise dedans pour left et right
    front_data = list(data.ranges[0:21])
    left_data = list(data.ranges[0:41])
    right_data = list(data.ranges[320:359])


    #print(f"Front : {front[0]}") 
    #print(f"Left : {left[35]}")
    #print(f"Right : {right[15]}")
    
    # pretraitement necessaire dans le cas ou le robot trop proche du mur envoie un inf  
    for i in range(len(front_data)):
        if str(front_data[i]) == "inf":
            front_data[i] = data.range_max
    
    for i in range(len(left_data)):
        if str(left_data[i]) == "inf":
            left_data[i] = data.range_max

    for i in range(len(right_data)):
        if str(right_data[i]) == "inf":
            right_data[i] = data.range_max

    tunnel_control(left_data,right_data)
    
def tunnel_control(left,right):
    coeff = 1
    ierror = 0
    linear_speed,angular_speed = 0,0
    # verifier si la distance future right et left fonctionne 
    right_point = right[33] # 33
    left_point = left[17] # 17
    front = right[0]
    if right_point == 3.5 or left_point == 3.5:
        coeff = 10
    else:
        coeff = 1
    pos = Twist()
    
    center = 0.75
    #print((right_point + left_point)/2)
    
    error = center - (right_point + left_point)/2
    kp = 0.04
    ki = 0.0175
    kd = 0.0275
    previous_error = 0
    ierror += error
    dt = 0.01
    control = kp  * error + kd*(error - previous_error) / dt + ki*(ierror)
    linear_speed = 0.15 - (np.abs(control)/1000 * 0.5 )
    angular_speed = -control/coeff
    
    if left_point >= 0.3 and right_point >= 0.3 and front >= 0.3:
        pos.linear.x = 0.3
        pos.angular.z = 0
        #rp.loginfo("Go straight")
    elif left_point < 0.6 and right_point > 0.6:
        pos.linear.x = linear_speed
        pos.angular.z = angular_speed
        rp.loginfo("droite")
    elif right_point < 0.6 and left_point > 0.6:
        pos.linear.x = linear_speed
        pos.angular.z = angular_speed
        rp.loginfo("gauche")
    else:
        pos.linear.x = linear_speed
        pos.angular.z = angular_speed
    pub.publish(pos)
    previous_error = error
    print ('-------------------------------------------')
    print ('Range data at 0 deg:   {}'.format(front))
    print ('Range data at 15 deg:  {}'.format(left[25]))
    print ('Range data at 345 deg: {}'.format(right[17]))
    print ('-------------------------------------------')
    print("Error : ",error)
    print("Linear speed : ",linear_speed)
    print("Angular speed :", angular_speed)



if __name__ == '__main__':


    rp.init_node('lidar_node',anonymous=True)
    rp.loginfo_once('Lidar node started')
    rp.Subscriber('/scan',LaserScan,callback=lidar_callback)
    #pub_lidar = rp.Publisher('/lidar_data',Float32MultiArray,queue_size=10)
    pub = rp.Publisher('/cmd_vel',Twist,queue_size=10)
    try: 
        
        rp.spin()
    except KeyboardInterrupt:
        print('Close')