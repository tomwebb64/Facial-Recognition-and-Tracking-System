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

face_cascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml") #loads pretrained classifier from file


bridge = CvBridge()
stop = False
img = None

no_face = False


# Runs when new data is recieved from the camera in gazebo
def camera_callback(data): 
    process_image(data)

# Begins node to read camera data and publish coordinates
def startNode():
    global face_pub
    rospy.init_node('read_image', anonymous=False)  
    rospy.Subscriber(cam_image_topic, CompressedImage, camera_callback) 
    face_pub = rospy.Publisher('face_boxes', Int16MultiArray, queue_size = 1)
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

    #print("Found {0} faces!".format(len(faces)))

    # Draw a rectangle around the faces
    #for (x, y, w, h) in faces:
    #	cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
    return faces

def publish_face(faces):
    faces_joined = []
    faces_len = len(faces)
    
    for (x, y, w, h) in faces:
        faces_joined.append(x)
        faces_joined.append(y)
        faces_joined.append(w)
        faces_joined.append(h)
    
    # Creating Template
    int_array = Int16MultiArray()

    int_array.data = faces_joined
    #rospy.loginfo("sent")
    face_pub.publish(int_array)
    


def process_image(img_ros):
    global no_face
    img_cv = bridge.compressed_imgmsg_to_cv2(img_ros)

    faces = get_face(img_cv)
    if len(faces) > 0:
        no_face = False
        publish_face(faces)
    else:
        if no_face == False:
            publish_face(faces)
            no_face = True
    
    

    #img = cv2.rotate(img_unrot, cv2.ROTATE_180)
'''
    cv2.imshow('frame', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
'''

        

startNode()
    




