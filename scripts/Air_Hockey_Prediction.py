#!/usr/bin/env python
import rospy, roslib, sys, cv2, os
from std_msgs.msg import Int16MultiArray 
import numpy as np
import math as m
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospkg import RosPack
rp = RosPack()
path = rp.get_path('air_hockey')

frame = np.zeros((480,640,3), np.uint8)
pts1 = np.float32([[0,0],[0,480],[640,0],[480,640]])
lower_puck = np.array([40,33,0])
upper_puck = np.array([93,182,255])
dilate_x = 1
virtual = cv2.imread(path+"/scripts/Template_Virtual_AH.jpg")
virtual_copy = virtual.copy()

end = [150.0,500.0]
start = [150.0,600.0]

arm_1 = [151,0,0,0,0]
arm_2 = [151,0,0,0,0]

I_puck = [0,0]
II_puck = [0,0]
III_puck = [0,0]

class air_hockey:
  def __init__(self):
    self.bridge = CvBridge()    
    self.image_rgb = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback_frame)
    #self.image_rgb = rospy.Subscriber("/video/AH",Image,self.callback_frame)
    self.pers_co = rospy.Subscriber("/air_hockey/pers_co",Int16MultiArray ,self.callback_pers_co)
    self.pub_perspective = rospy.Publisher('/air_hockey/perspective', Image, queue_size=1)
    self.pub_puck = rospy.Publisher('/air_hockey/puck', Image, queue_size=1)
    self.pub_circle = rospy.Publisher('/air_hockey/circle', Image, queue_size=1)
    self.pub_cms = rospy.Publisher('/air_hockey/position', Int16MultiArray, queue_size=1)
    self.color = rospy.Subscriber("/air_hockey/color",Int16MultiArray ,self.callback_color)
    
  def callback_frame(self,data):
    global frame
    frame = self.bridge.imgmsg_to_cv2(data,"bgr8")

  def callback_pers_co(self,ros_data):
    global pts1
    data = ros_data.data
    pts1 = np.float32([[data[0],data[1]],[data[2],data[3]],[data[4],data[5]],[data[6],data[7]]])

  def callback_color(self,ros_data):
    global lower_puck, upper_puck, dilate_x
    data = ros_data.data
    lower_puck = np.array([data[0],data[1],data[2]])
    upper_puck = np.array([data[3],data[4],data[5]])
    dilate_x = data[6]
    
def perspective():
  global frame,pts1
  pts2 = np.float32([[300,0],[0,0],[300,600],[0,600]])
  #pts2 = np.float32([[0,0],[0,600],[300,0],[600,300]])
  M = cv2.getPerspectiveTransform(pts1,pts2)
  arena = cv2.warpPerspective(frame,M,(300,600))
  air_hockey.pub_perspective.publish(air_hockey.bridge.cv2_to_imgmsg(arena,"bgr8"))
  return arena
  
def mask(arena):
  hsv_arena = cv2.cvtColor(arena, cv2.COLOR_BGR2HSV)
  puck = cv2.inRange(hsv_arena, lower_puck, upper_puck)
  puck = cv2.erode(puck, np.ones((2,2),np.uint8),iterations = dilate_x)
  puck = cv2.dilate(puck, np.ones((2,2),np.uint8),iterations = dilate_x)
  puck = cv2.cvtColor(puck, cv2.COLOR_GRAY2BGR)
  air_hockey.pub_puck.publish(air_hockey.bridge.cv2_to_imgmsg(puck,"bgr8"))
  return puck

def detect(arena,puck):
  global y_prev,virtual,virtual_copy,end
  puck = cv2.cvtColor(puck, cv2.COLOR_BGR2GRAY)
  circles = cv2.HoughCircles(puck,cv2.cv.CV_HOUGH_GRADIENT,1,5,param1=2,param2=2,minRadius=8,maxRadius=12)
  if circles is not None:
    for i in circles[0,:]:
      virtual = virtual_copy.copy()
      cv2.circle(virtual,(i[0],i[1]),i[2],(0,255,0),-1)
      pos = [i[0],i[1]]
      x = int((i[0]*0.3048))
      y = int((-i[1]+300)*0.3048)
      xy = np.asarray([x,y])
      air_hockey.pub_cms.publish(data = xy.tolist())
      break
    return pos
    air_hockey.pub_circle.publish(air_hockey.bridge.cv2_to_imgmsg(virtual,"bgr8"))

#def check_slope(
if __name__ == '__main__':
  rospy.init_node('air_hockey', anonymous=True)
  air_hockey = air_hockey()
  while not rospy.is_shutdown():
    rate = rospy.Rate(60)
    arena = perspective()
    puck = mask(arena)
    I_puck = detect(arena,puck)
    check_slope(I_puck)
    rate.sleep()
