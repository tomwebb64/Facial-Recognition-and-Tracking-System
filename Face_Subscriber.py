#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import colorsys
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge #Used to convert from ros string image to matrix image
from geometry_msgs.msg import Pose2D
from time import sleep
#import Face_Recognition_Ros #Import other python script
import face_recognition

cam_image_topic = "/raspicam_node/image/compressed" # change for each device

face_topic = "/face_boxes" # change namespace for each device

face_image_topic = "/face_image_cropped"

bridge = CvBridge()
stop = False
img = None

close_preview = False

frames_since_last_face = 0
process_this_frame = True

obama_image = face_recognition.load_image_file("obama.jpg")
obama_face_encoding = face_recognition.face_encodings(obama_image)[0]

# Load a second sample picture and learn how to recognize it.
biden_image = face_recognition.load_image_file("biden.jpg")
biden_face_encoding = face_recognition.face_encodings(biden_image)[0]

tom_image = face_recognition.load_image_file("Tom.png")
tom_face_encoding = face_recognition.face_encodings(tom_image)[0]

# Create arrays of known face encodings and their names
known_face_encodings = [
    obama_face_encoding,
    biden_face_encoding,
    tom_face_encoding
]



known_face_names = [
    "Barack Obama",
    "Joe Biden",
    "Tom Webb"
]


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

def face_img_callback(face_img_ros):
    global bridge
    global close_preview
    face_img_cv = bridge.imgmsg_to_cv2(face_img_ros)
    if close_preview == False:
        cv2.imshow('face', face_img_cv)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        close_preview = True
        cv2.destroyAllWindows()

# Begins node to read camera data and publish coordinates
def startNode():
    global face_pub
    global name_pub
    rospy.init_node('read_faces', anonymous=False)  
    rospy.Subscriber(cam_image_topic, CompressedImage, image_callback) 
    rospy.Subscriber(face_topic, Int16MultiArray, face_callback)
    rospy.Subscriber(face_image_topic, Image, face_img_callback) 
    face_pub = rospy.Publisher('face_boxes', Int16MultiArray, queue_size = 10)
    name_pub = rospy.Publisher('names', String, queue_size=10)
    rate = rospy.Rate(50) 
    rospy.spin() # Keep python running the whole while this node is active.


def find_name(face_crop_list):
    global process_this_frame
    global known_face_encodings
    global known_face_names
    for frame in face_crop_list:
        
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = small_frame[:, :, ::-1]

        # Only process every other frame of video to save time
        if process_this_frame == 30:
            process_this_frame = 0
            # Find all the faces and face encodings in the current frame of video

            face_locations = face_recognition.face_locations(rgb_small_frame)
            face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

            face_names = []
            #print (face_encodings)
            for face_encoding in face_encodings:
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                name = "Unknown"

                # Uses the known face with the smallest distance to the new face
                face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                best_match_index = np.argmin(face_distances)
                if matches[best_match_index]:
                    name = known_face_names[best_match_index]
                    


                face_names.append(name)
            return(face_names)

        process_this_frame += 1

        # Display the results
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4

            # Draw a box around the face
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

            # Draw a label with a name below the face
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
    #print ("name: ",face_names)


def process_image(img_ros):
    global close_preview
    global name_pub
    # change values to increase border around face when cropping
    y_offset = 10 
    x_offset = 10
    names_list = []

    img_cv = bridge.compressed_imgmsg_to_cv2(img_ros)
    face_crop_list = []
    try:
        if len(face_list)>0:
            for (x, y ,w, h) in face_list:
                middle_coordx, middle_coordy = x+(w/2),y+(h/2)
                
                img_cv = cv2.rectangle(img_cv,(x,y),(x+w,y+h),(0,255,0),2)
                
                face_crop_list.append(img_cv[y-y_offset:y+h+y_offset, x-x_offset:x+w+x_offset]) # adds each face as a cropped image, to a list
            
            names_list = find_name(face_crop_list)

            ######MESAGE NOT SENDING#########
            try:
                message = String(str(names_list[0])+"="+str(middle_coordx)+","+str(middle_coordy))
                #message.data = (str(names_list[0]),"=",str(middle_coord))
                print(message)
                name_pub.publish(message)
            except IndexError:
                print("Could not recognise face")

    except NameError:
        None
    # img = cv2.rotate(img_unrot, cv2.ROTATE_90_CLOCKWISE)
    if close_preview == False:
        cv2.imshow('frame', img_cv)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        close_preview = True
        cv2.destroyAllWindows()



startNode()
    



