#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import rospy as rp
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

front_data = []
right_data = []
left_data = []
angle_left = []
angle_right = []
def lidar_callback(data):
    global front_data
    global left_data
    global right_data

    global angle_left
    global angle_right
    # front_data = list(data.ranges[0:21])
    # left_data = list(data.ranges[0:90])
    # right_data = list(data.ranges[269:359])
    front_data = list(data.ranges[0:21])
    left_data = list(data.ranges[0:180])
    right_data = list(data.ranges[180:359])
    
    #front_data = list(data.ranges[0:21])
    #left_data = list(data.ranges[0:41])
    #right_data = list(data.ranges[320:359])
    
    angle_min = data.angle_min 
    angle_incre = data.angle_increment

    ray_angle = []
    for i in range(len(data.ranges)):
        ray_angle.append(angle_min + (i*angle_incre))

    angle_left = ray_angle[0:90]
    angle_right = ray_angle[270:359]
    #print(f"Front : {front[0]}") 
    #print(f"Left : {left[35]}")
    #print(f"Right : {right[15]}")
    
    for i in range(len(front_data)):
        if str(front_data[i]) == "inf":
            front_data[i] = data.range_max
    
    for i in range(len(left_data)):
        if str(left_data[i]) == "inf":
            left_data[i] = data.range_max

    for i in range(len(right_data)):
        if str(right_data[i]) == "inf":
            right_data[i] = data.range_max


def left_callback(data):

    global right_cx
    global left_cx
    global height 
    global width 
    left_cx = data.data[0]
    right_cx = data.data[1]
    height = data.data[2]
    width = data.data[3]
    global lane_state
    lane_state = True
    if left_cx == height//2 +45 and right_cx == height//2 +45 : 
        lane_state = False
    else:
        lane_state = True
    control()
    #rp.loginfo("Left_centroid : {0}, Right_centroid : {1}".format(240//2 + 45 - left_cx,240//2 + 45 - right_cx))


################################################## Obstacle avoidance for the line ###############################################
def obstacle_avoidance(front,left,right,angle_left,angle_right):
    dependency = "none"
    threshold = 0.2
    pos = Twist()
    # recuperer la position des obstacles 
    if np.min(left) < threshold and  np.min(front)< 0.45 :
        dependency = "left"
    elif np.min(right) < threshold:
        dependency = "right"
    elif front[0] > 0.45 and np.min(left) < threshold:
        dependency = "straight"
    else:
        dependency = "none"

    if dependency == "left":
        pos.linear.x = 0
        pos.angular.z = 1.57

    if dependency == "straight":
        pos.linear.x = 0.15
        pos.angular.z = 0

    if dependency == "right":
        if front[0] > 0.45:
            pos.linear.x = 0.15
            pos.angular.z = 0
        else:
            pos.linear.x = 0
            pos.angular.z = -1.57

    if dependency == "none":
        pos.linear.x = 0
        pos.angular.z = 0

    print("Left: ",np.min(left))
    print("Right:",np.min(right))
    print("Front :",front[0])
    print("Dependcy :",dependency)
    print("-----------------------------------")
    pub.publish(pos)


########################################################## Tunnel ##################################################
def tunnel_control(left,right):
    coeff = 1
    ierror = 0
    linear_speed,angular_speed = 0,0
    # verifier si la distance future right et left fonctionne 
    right_point = right[33] # 33
    left_point = left[23] # 17
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
        pos.angular.z = -angular_speed
        rp.loginfo("droite")
    elif right_point < 0.6 and left_point > 0.6:
        pos.linear.x = linear_speed
        pos.angular.z = angular_speed
        rp.loginfo("gauche")
    else:
        pos.linear.x = linear_speed
        pos.angular.z = angular_speed

    print ('-------------------------------------------')
    print ('Range data at front:   {}'.format(front))
    print ('Range data at left:  {}'.format(left_point))
    print ('Range data at right: {}'.format(right_point))
    print ('-------------------------------------------')
    print("Error : ",error)
    print("Linear speed : ",linear_speed)
    print("Angular speed :", angular_speed)
    pub.publish(pos)
    previous_error = error

############################################################# Lane control ##################################################

def control_lane(left,right,height,width):
     # initial parameters 
    linear_speed, angular_speed = 0,0
    centre_lane = [width//2+45,height//2+45]
    state = "both"
    pos = Twist()
    # PID coefficients values 
    kp = 0.05
    ki = 0.0175
    kd = 0.0225
    dt = 0.01
    
    # dans le cas où on prend une des lignes dependances du ligne présent
    if (centre_lane[1] - int(left)) == 0: #ligne jaune perdu -> dependance du ligne blanche
        angular_error = (centre_lane[1] - right) 
        state = "right"
        #print("left lane perdu ")
    if (centre_lane[1] - int(right)) == 0: #ligne blanche perdu -> dependance du ligne jaune
        angular_error = centre_lane[1] - left
        #print("right lane perdu ")
        state ="left"
    else:
        angular_error = centre_lane[1] - ((left + right)//2)

    previous_error = 0
    ierror = 0
    ierror += angular_error
    dt = 0.01
    control = kp  * angular_error + kd*(angular_error - previous_error) / dt + ki*(ierror)
    linear_speed = 0.15 - (np.abs(control)/1000* 0.5 )
    angular_speed = control/100


    if state == "right":
        kp = 0.05
        ki = 0.0175
        kd = 0.035
        dt = 0.01
        previous_error = 0
        ierror = 0
        ierror += (angular_error+38)
        dt = 0.01
        control = kp  * (angular_error+38) + kd*((angular_error+38) - previous_error) / dt + ki*(ierror)
        linear_speed = 0.15 - (np.abs(control)/1000* 0.5 )
        angular_speed = control/100
        if angular_error < -41: # 26 car le chemin est tres large et normalement sur une route normale la distance entre le centre
            # et le centroid rouge est de -84
            pos.linear.x = linear_speed
            pos.angular.z = angular_speed
            print("droite")
        else:
            pos.linear.x = linear_speed
            pos.angular.z = angular_speed
            print("gauche")
    
    if state == "left":
        kp = 0.05
        ki = 0.0175
        kd = 0.035
        dt = 0.01
        previous_error = 0
        ierror = 0
        ierror += (angular_error+32)
        dt = 0.01
        control = kp  * (angular_error+32) + kd*((angular_error+32) - previous_error) / dt + ki*(ierror)
        linear_speed = 0.15 - (np.abs(control)/1000* 0.5 )
        angular_speed = -control/100
        if angular_error < 41: 
            pos.linear.x = linear_speed
            pos.angular.z = angular_speed
            print("droite")
        else:
            pos.linear.x = linear_speed
            pos.angular.z = angular_speed
            print("gauche")
    
    print("Line state : ",state)
    print("Angular error :",angular_error)
    print("Control output :",control)
    print("Linear speed : ", linear_speed)
    print("Angular speed :", angular_speed)
    print("-------------------------------")
    print("Left centroid : ",left)
    print("Right centroid :",right)
    print("Center to left: ",centre_lane[1] - left)
    print("Center to right : ",centre_lane[1] - right)
    print("-------------------------------")
    
    # cette condition permet de entrer dans le rondpoint par droite et de sortir 
    if (centre_lane[1] - left) > 84 and (centre_lane[1] - right) < -84:
        pos.linear.x = linear_speed
        pos.angular.z = -0.3
    else:
        pos.linear.x = linear_speed
        pos.angular.z = angular_speed 

    
    pub.publish(pos)
    previous_error = angular_error # l'erreur précedent prend l'erreur courant 


################################ Control Loop #########################################
def control():
    threshold = 0.38
    pos = Twist()
    if rp.has_param("challenge"):
        challenge = rp.get_param("challenge")
    else:
        challenge = "challenge1"

    if challenge =="challenge1":
        if front_data[0] > threshold and lane_state == True:
            #print(front_data[0])
            #print("Lane control")
            control_lane(left_cx,right_cx,height,width)
        elif front_data[0] < threshold or lane_state == False:
            obstacle_avoidance(front_data,left_data,right_data,angle_left,angle_right)
        
    if challenge == "challenge2":
        if front_data[0] > threshold and lane_state == True:
                #print(front_data[0])
                #print("Lane control")
                control_lane(left_cx,right_cx,height,width)
        elif lane_state == False:
            #print("Tunnel")
            tunnel_control(left_data[0:41],right_data[140:])
        elif np.min(front_data) < 0.4 and lane_state == False:
            pos.linear.x = 0
            pos.angular.z = 0 
            pub.publish(pos)

if __name__ == '__main__':
    rp.init_node('control_node',anonymous=True)
    rp.loginfo_once('Control node started....................')
    rp.Subscriber('/scan',LaserScan,callback=lidar_callback)
    rp.Subscriber('/centroids',Float32MultiArray,callback=left_callback) # the centroids and other data given by the image processing node
    
    pub = rp.Publisher('/cmd_vel',Twist,queue_size=10)
   
    #control_lane(right_cx,left_cx)
    try: 
            
        rp.spin()
    except KeyboardInterrupt:
        print('Close')
