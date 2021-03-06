from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import os
from robot_interface import robot_interface
import time

robot_interface = robot_interface()

camera = PiCamera()
camera.resolution = (640, 480)#(640, 480) #(1920, 1088)
camera.framerate = 30#32
rawCapture = PiRGBArray(camera, size=(640, 480)) #(640, 480) #(1920, 1088)

face_cascade = cv2.CascadeClassifier()
face_cascade.load("/home/eddy/code/haarcascade_frontalface_default.xml")

time.sleep(0.1)

print("starting!")
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    
    if len(faces) > 0:
    	x, y, w, h = faces[0]
    	print("x:", x - 240, "y:", y);
    	robot_interface.set_velocity(40.0, -(x - 240)/240)
    else:
    	robot_interface.set_velocity(0.0, 0.0)
    	
    #for (x,y,w,h) in faces:
    	#cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)
    	#print("x:", x-240, "y:", y);

    #cv2.imshow("faces", image)
    #key = cv2.waitKey(1)

    rawCapture.truncate(0)
    
    #if key == ord("q"):
        #break