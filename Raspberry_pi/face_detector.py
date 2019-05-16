from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import os

camera = PiCamera()
camera.resolution = (640, 480)#(640, 480) #(1920, 1088)
camera.framerate = 30#32
rawCapture = PiRGBArray(camera, size=(640, 480)) #(640, 480) #(1920, 1088)

face_cascade = cv2.CascadeClassifier()
face_cascade.load("/home/eddy/code/haarcascade_frontalface_default.xml")

time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    
    for (x,y,w,h) in faces:
    	cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)

    cv2.imshow("faces", image)
    key = cv2.waitKey(1)

    rawCapture.truncate(0)
    
    if key == ord("q"):
        break
