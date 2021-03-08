#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import colorsys
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge #Used to convert from ros string image to matrix image
from geometry_msgs.msg import Pose2D
from time import sleep


cam_image_topic = "/raspicam_node/image/compressed" # change for each device

face_topic = "/face_boxes" # change namespace for each device


bridge = CvBridge()
stop = False
img = None

close_preview = False

Frames_since_last_face = 0


# Runs when new data is recieved from the camera
def image_callback(img_ros): 
    global frames_since_last_face
    process_image(img_ros)
    frames_since_last_face += 1
    #print ("Frames_since_last_face", Frames_since_last_face)

def face_callback(face_data):
    global face_list
    global frames_since_last_face
    faces = face_data.data
    face_list = []
    face_list = ([faces[i:i+4] for i in range(0, len(faces), 4)])
    #print (len(face_list),"faces Detected")
    frames_since_last_face = 0

# Begins node to read camera data and publish coordinates
def startNode():
    global face_pub
    rospy.init_node('read_faces', anonymous=False)  
    rospy.Subscriber(cam_image_topic, CompressedImage, image_callback) 
    rospy.Subscriber(face_topic, Int16MultiArray, face_callback)
    face_pub = rospy.Publisher('face_boxes', Int16MultiArray, queue_size = 10)
    rate = rospy.Rate(50) 
    rospy.spin() # Keep python running the whole while this node is active.




def process_image(img_ros):
    global close_preview
    img_cv = bridge.compressed_imgmsg_to_cv2(img_ros)
    try:
        if len(face_list)>0:
            for (x, y ,w, h) in face_list:
                img_cv = cv2.rectangle(img_cv,(x,y),(x+w,y+h),(0,255,0),2)
    except NameError:
        None
    # img = cv2.rotate(img_unrot, cv2.ROTATE_90_CLOCKWISE)
    if close_preview == False:
        cv2.imshow('frame', img)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        close_preview = True
        cv2.destroyAllWindows()



startNode()
    



