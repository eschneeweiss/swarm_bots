#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import os
import time

pub = rospy.Publisher('velocity', Float64,queue_size = 10)
rospy.init_node('camera', anonymous=True)

rate = rospy.Rate(30)

camera = PiCamera()
camera.resolution = (640, 480)#(640, 480) #(1920, 1088)
camera.framerate = 30#32
rawCapture = PiRGBArray(camera, size=(640, 480)) #(640, 480) #(1920, 1088)

face_cascade = cv2.CascadeClassifier()
face_cascade.load("/home/eddy/code/haarcascade_frontalface_default.xml")

time.sleep(0.1)

print("starting!")
while not rospy.is_shutdown():
	print ("rospy is not down")
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		image = frame.array
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		faces = face_cascade.detectMultiScale(gray, 1.3, 5)
		if len(faces) > 0:
			x, y, w, h = faces[0]
			print("x:", x - 240, "y:", y)
			rospy.loginfo(x)
			pub.publish(x)
			rate.sleep()
		else:
			rospy.loginfo(0)
			pub.publish(0)
			rate.sleep()
		rawCapture.truncate()
		rawCapture.seek(0)
		




