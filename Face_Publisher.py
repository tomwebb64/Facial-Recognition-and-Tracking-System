#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import colorsys
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge #Used to convert from ros string image to matrix image
from geometry_msgs.msg import Pose2D
from time import sleep


cam_image_topic = "/raspicam_node/image/compressed" # change for each device

face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml") #loads pretrained classifier from file


bridge = CvBridge()
stop = False
img = None

no_face = False


# Runs when new data is recieved from the camera
def camera_callback(data): 
    process_image(data)

# Begins node to read camera data and publish coordinates
def startNode():
    global face_pub
    global face_img_pub

    rospy.init_node('read_image', anonymous=False)  
    rospy.Subscriber(cam_image_topic, CompressedImage, camera_callback) 
    face_pub = rospy.Publisher('face_boxes', Int16MultiArray, queue_size = 1)
    face_img_pub = rospy.Publisher("face_image_cropped",Image, queue_size = 2)
    rate = rospy.Rate(50) 
    rospy.spin() # Keep python running the whole while this node is active.

def get_face (img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # converts 3 channel colour image into 1 chanel image

    # Detects faces in the grey image using classifier
    faces = face_cascade.detectMultiScale(
        gray,
        scaleFactor=1.1,
        minNeighbors=5,
        minSize=(30, 30)
        #flags = cv2.CV_HAAR_SCALE_IMAGE
    )

    return faces

def publish_face(faces, img_cv):
    global bridge
    faces_joined = []
    face_img_list = []
    faces_len = len(faces)
    y_offset =10 
    x_offset = 10
    
    for (x, y, w, h) in faces:
        crop_img = img_cv[y-y_offset:y+h+y_offset, x-x_offset:x+w+x_offset]
        # face_img_list.append(crop_img)
        crop_img_ros = bridge.cv2_to_imgmsg(crop_img, encoding="passthrough")
        face_img_pub.publish(crop_img_ros)

        faces_joined.append(x)
        faces_joined.append(y)
        faces_joined.append(w)
        faces_joined.append(h)
    
    # Creating Template
    int_array = Int16MultiArray()
    # int_array.stamp = rospy.Time.now()
    # print (int_array.stamp)
    int_array.data = faces_joined
    face_pub.publish(int_array)

    
    


def process_image(img_ros):
    global no_face
    img_cv = bridge.compressed_imgmsg_to_cv2(img_ros)

    faces = get_face(img_cv)
    if len(faces) > 0:
        no_face = False
        publish_face(faces, img_cv)
    else:
        if no_face == False:
            publish_face(faces, img_cv)
            no_face = True
    
    

    #img = cv2.rotate(img_unrot, cv2.ROTATE_180)


        

startNode()
    




