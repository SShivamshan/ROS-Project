#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import rospy as rp
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge,CvBridgeError # covertie les données venant de cv2 à des message ros
import numpy as np
from std_msgs.msg import Float32MultiArray
from queue import Queue


def getcontours_left(img):
    global prev_left
    prev_left = [78,205]
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #rp.loginfo_once("{0}".format(contours))
    cX_final,cY_final =0,0
    cX,cY = 0,0 
    h,w= img.shape[0],img.shape[1]
    centre  = [w//2 + 45 ,h//2  +45]
    
    if len(contours) > 0:
        c = max(contours,key=cv2.contourArea)
        moment = cv2.moments(c)
        # trouver le centre de l'objet
        if (moment["m00"]) != 0:
            cX = (int(moment["m10"] / moment["m00"]))
            #cX.append(int(moment["m10"] / moment["m00"]))
            cY = (int(moment["m01"] / moment["m00"]))
            #cY.append(int(moment["m01"] / moment["m00"]))
            #rp.loginfo(" Right : {0}, {1}".format(cX,cY))
            #rp.loginfo(" len Cx: {0} ".format(len(cX)))
        else:
            rp.loginfo_once("Division par zero pour left lane")
            # dans ce cas les centroids prennent les valeurs des centroids précedents pour ne pas perdre complétement la ligne
            cX_final, cY_final = prev_left[0],prev_left[1]
        cX_final = cX
        cY_final = cY
    else:
        cX_final, cY_final = centre[1],centre[0]
        #prev_left[0],prev_left[1] = centre[1],centre[0]
        rp.loginfo_once("Line left not found.............")
    #rp.loginfo("{0},{1}".format(cX,cY))

    #rp.loginfo(" Left : {0}, {1}".format(cX_final,cY_final))
    prev_left[0],prev_left[1] = cX_final,cY_final
    return cX_final,cY_final



def getcontours_right(img):
    
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #rp.loginfo_once("{0}".format(contours))
    cX_final,cY_final =0,0
    cX,cY =0,0
    h,w= img.shape[0],img.shape[1]
    global prev_right
    prev_right = [247,205]
    centre  = [w//2 + 45 ,h//2  +45]
    
    if len(contours) > 0:
        ##rp.loginfo_once("{0}".format(count))
        c = max(contours,key=cv2.contourArea)
        moment = cv2.moments(c)
        # trouver le centre de l'objet
        if (moment["m00"]) != 0:
            cX = (int(moment["m10"] / moment["m00"]))
            #cX.append(int(moment["m10"] / moment["m00"]))
            cY = (int(moment["m01"] / moment["m00"]))
        else:
            rp.loginfo_once("Division par zero pour right lane")
            cX_final, cY_final = prev_right[0],prev_right[1]
        cX_final = cX
        cY_final = cY
    else:
        cX_final, cY_final = centre[1],centre[0]
        rp.loginfo_once("Line right not found.............")
    
    #rp.loginfo(" Right : {0}, {1}".format(cX_final,cY_final))
    prev_right[0],prev_right[1] = cX_final,cY_final
    return cX_final,cY_final


# pour la réduction de l'intensité lumineuse 
def gamma(img,gam):
    inv = 1/gam
    
    table = [((i / 255) ** inv) * 255 for i in range(256)]
    table = np.array(table, np.uint8)
    return cv2.LUT(img,table)  


def img_callback(msg,args):
    if not args.full():
        args.put(msg)
    else:
        rp.logerr_once("Pas d'information reçu car buffer rempli")

    try:
        img_processing(args)
        #rp.loginfo("Here")
    except CvBridgeError as e:
        rp.logerr_once("Conversion img message a cv2 pas fait ") 


def img_processing(args):
    if not args.empty():
        msg = args.get()
        np_arr = np.frombuffer(msg.data,np.uint8)
        image_np = cv2.imdecode(np_arr,cv2.IMREAD_COLOR)
        h,w,c = image_np.shape[0],image_np.shape[1],image_np.shape[2]
        # reduction de l'image a une region d'intéret 
        #polygon = np.array([[w+100,w],[h,160],[120,160],[30,h]])
        polygon = np.array([[w,w],[h,165],[120,165],[30,h]]) # pour la simulation 
        # polygon = np.array([[w+270,w],[h,250],[120,250],[-200,w]]) # pour le vrai robot 
        mask = np.zeros_like(image_np)
        match_mask_color = (255,) * c
        cv2.fillConvexPoly(mask, polygon, match_mask_color)
        masked_image = cv2.bitwise_and(image_np, mask)

        imgblur = cv2.GaussianBlur(masked_image, (5,5), 0)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        opening = cv2.morphologyEx(imgblur,cv2.MORPH_OPEN,kernel)
        closing = cv2.morphologyEx(opening,cv2.MORPH_CLOSE,kernel)
        # on réduit l'intensité luminueuse
        # tres important pour le traitement d'image du robot réel due aux variations des intensité lumineuse
        imggamma = gamma(closing,0.5)

        # transformation de l'image BGR en HSV
        imageHSV = cv2.cvtColor(imggamma, cv2.COLOR_BGR2HSV)
        # pour le jaune en simulation  et vert dans la course left lane 
        #lower_jaune = np.array([0,45,4])
        #higher_jaune = np.array([47,255,255])
        lower_jaune = np.array([20,100,100])
        higher_jaune = np.array([50,255,255])

        mask_jaune = cv2.inRange(imageHSV, lower_jaune, higher_jaune)
        #lower_vert = np.array([0,26,0])
        #higher_vert = np.array([166,255,255])
        #mask_vert = cv2.inRange(imageHSV, lower_vert, higher_vert)

        # pour le rouge et le blanc(simulation) right lane
        #lower_rouge = np.array([134,30,0])
        #higher_rouge = np.array([179,255,255])
        #mask_rouge = cv2.inRange(imageHSV, lower_rouge, higher_rouge)
        #lower_blanc = np.array([0,0,3])
        #higher_blanc = np.array([179,0,255])
        lower_blanc = np.array([0,0,155])
        higher_blanc = np.array([255,30,255])
        mask_blanc = cv2.inRange(imageHSV, lower_blanc, higher_blanc)

        mask = mask_jaune + mask_blanc # on ajoute les 2 couleurs 
        masked_image = cv2.bitwise_and(masked_image,masked_image,mask=mask)
        # on réduit le bruit et ferme les trous de petites tailles 

        cX_rouge,cY_rouge = getcontours_right(mask_blanc) # rouge,blanc, right lane
        cX_vert,cY_vert = getcontours_left(mask_jaune) # jaune, vert, left lane 
        imgfinal = cv2.circle(masked_image, (cX_rouge, w//2+45), 5, (0, 0,255), -1)
        imgfinal = cv2.circle(masked_image, (cX_vert, w//2+45), 5, (0, 255,0), -1)
        imgfinal = cv2.circle(masked_image, ((cX_rouge + cX_vert)//2, w//2+45), 3, (255, 0,0), -1) # simulation 
    
        cv2.imshow("Image finale",imgfinal)
        cv2.imshow("Mask",mask)
        if cv2.waitKey(1) & 0xFF == ord('h'):
            cv2.destroyAllWindows()
        queue.task_done()

        # on publie les centroids et la valeur de la taille de l'image
        centroid_values = Float32MultiArray()
        centroid_values.data = [cX_vert,cX_rouge,h,w]
        pub_centroid.publish(centroid_values)
        

if __name__ == '__main__':

    rp.init_node('Camera_node',anonymous=True)
    rp.loginfo_once('Camera node started.........')
    # /camera/image/compressed
    queue = Queue(maxsize=1)
    rp.Subscriber('/camera/image/compressed',CompressedImage,img_callback,(queue))
    pub_centroid = rp.Publisher('/centroids',Float32MultiArray,queue_size=10)
    try: 
            
        rp.spin()
    except KeyboardInterrupt:
        print('Close')