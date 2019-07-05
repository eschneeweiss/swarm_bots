#!/usr/bin/env python3
import rospy
import os
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from robotInterface import robotInterface

a = False
b = False
c = False

robotInterface = robotInterface()

print ("Hello world")

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	[a, b, c] = robotInterface.read_buttons()
	
		
	if data.data > 0:
		robotInterface.set_velocity(40.0, -(data.data - 240)/240)
		a = False
		b = True
		robotInterface.leds(a,b,c)
	else:
		robotInterface.set_velocity(0.0,0.0)
		a = True
		b = False
		robotInterface.leds(a,b,c)

def listener():
	rospy.init_node("robot_base", anonymous=True)
	rospy.Subscriber("velocity", Float64, callback)

	
	rospy.spin()
	
print ("passed listener")
if __name__ == '__main__':
	listener()
	print ('Hi')
	robotInterface.leds(a,b,c)

else:
	"name=/=main"

print ("Bye")