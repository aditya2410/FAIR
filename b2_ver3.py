from PIL import Image
import numpy as np
from pylab import *
import cv2
import rospy
import math
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose



cap=cv2.VideoCapture(1)
n=0
turtle_theta=0.0

rospy.init_node('move')
p=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
p1=rospy.Publisher('dhawan',Point,queue_size=10)
rate=rospy.Rate(15)
rate2=rospy.Rate(50)
center=[-1,-1]



def function1(bgr):
	hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
	#Threshold the hue image to get the ball.
	ball = cv2.inRange(hsv,np.array([126,161,108]),np.array([179,255,255]))
	kernel = np.ones((3,3),np.uint8)
	erosion = cv2.erode(ball,kernel,iterations = 1)
	moments = cv2.moments(erosion)
	
	cv2.imshow("ball",erosion)
	m00 = moments['m00']
	centroid_x, centroid_y = None, None
	if m00 != 0:
		centroid_x = int(moments['m10']/m00)
		centroid_y = int(moments['m01']/m00)
	
	# Assume no centroid
	centroid = Point(-1,-1,0)
	# Use centroid if it exists
	if (centroid_x != None and centroid_y != None):
		centroid = Point(centroid_x,centroid_y,0)
	return centroid

def function2(centroid,cur_centroid):
	x1=centroid.x
	y1=centroid.y
	x2=cur_centroid.x
	y2=cur_centroid.y
	d=math.sqrt((y2-y1)**2+(x2-x1)**2)
	return d

def angle(prev_centroid,centroid):
	x1=centroid.x
	y1=centroid.y
	x2=prev_centroid.x
	y2=prev_centroid.y
	dx=x1-x2
	d=math.sqrt((y2-y1)**2+(x2-x1)**2)
	theta=math.acos(dx/d)
	return (theta)

def callback1(data):
	global turtle_theta
	turtle_theta=data.theta
	#print turtle_theta

while not rospy.is_shutdown():
	ret,bgr=cap.read()
	bgr=cv2.flip(bgr,1)
	h,w,c=bgr.shape
	
	'''if(n>0):
		prev_centroid=centroid
		#changing the previous center to image center
		prev_centroid=Point(w/2,h/2,0)'''

	#get the centroid
	centroid=function1(bgr)
	
	if (n==0 and centroid.x!=-1):
		center=[centroid.x,centroid.y]
		n=n+1

	theta,d=None,None
	d=function2(Point(center[0],center[1],0),centroid)
	if (centroid.x!=-1 and d>15):
		theta=angle(Point(center[0],center[1],0),centroid)
		print theta
		cv2.circle(bgr,(centroid.x,centroid.y),5,(0,255,0),-1)
		
		#rotation defines the direction to rotate
		#choose according to convinience 
		rotation=0
		if (theta<0.15):
			rotation =d
			p1.publish(Point(rotation,0.0,0.0))
			rate.sleep()
		elif (theta>3.00):
			rotation=-1*d
			p1.publish(Point(rotation,0.0,0.0))
			rate.sleep()
		elif (theta>1.5 and theta<2.0 and centroid.y<center[1]):
			rotation =0
			p1.publish(Point(rotation,d,0.0))
			rate.sleep()





	'''if (n==0):
		center=[centroid.x,centroid.y]
		prev_centroid=centroid
		n=n+1

	#initialising the distance to zero
	d=0.0
	if (center[0]!=-1 and centroid.x!=-1):
		d=function2(Point(center[0],center[1],0),centroid)
		cv2.circle(bgr,(centroid.x,centroid.y),5,(0,255,0),-1)

	theta=0.0
	print d
	if (d>15):

		theta=angle(Point(center[0],center[1],0),centroid)


		if(prev_centroid.y<centroid.y):
			theta=2*3.141-theta
	
	rospy.Subscriber("/turtle1/pose",Pose,callback1)
	delta_theta=theta-turtle_theta

	if (delta_theta>3.141):
		delta_theta=3.141-delta_theta

	twist=Twist()
	if (delta_theta>3.141): 
		twist.linear.x=0.0
		twist.linear.y=0;twist.linear.z=0;
		twist.angular.x=0;twist.angular.y=0;
		twist.angular.z=delta_theta/4.0;
	else:
		twist.linear.x=0.0
		twist.linear.y=0;twist.linear.z=0;
		twist.angular.x=0;twist.angular.y=0;
		twist.angular.z=delta_theta;
	
	p.publish(twist)
	rate.sleep()
	if (delta_theta<0.1 and d>15):
		twist=Twist()
		twist.linear.x=d/30.0
		twist.linear.y=0;twist.linear.z=0;
		twist.angular.x=0;twist.angular.y=0;
		twist.angular.z=0.0;
		p.publish(twist)
		rate.sleep()

	#twist=Twist()
	#p.publish(twist)
	#print theta'''

	cv2.circle(bgr,(center[0],center[1]),15,(0,0,255),-1)
	cv2.imshow("original",bgr)
	k = cv2.waitKey(30) & 0xFF
	if k == 27:
		break

    
cap.release()
cv2.destroyAllWindows()
   
