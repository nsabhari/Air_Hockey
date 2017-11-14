#!/usr/bin/env python

import rospy,roslib,sys,cv2,serial,syslog,time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospkg import RosPack
rp = RosPack()
path = rp.get_path('air_hockey')

def video_play():
  rate = rospy.Rate(10)
  cap = cv2.VideoCapture(path+"/bagfiles/Air_Hockey_PS3.avi")
  pub_video = rospy.Publisher('/video/AH', Image, queue_size=1)
  bridge = CvBridge()
  while not rospy.is_shutdown():
    ret, frame = cap.read()
    if ret == False:
      cap = cv2.VideoCapture(path+"/bagfiles/Air_Hockey_PS3.avi")
      ret, frame = cap.read()
    pub_video.publish(bridge.cv2_to_imgmsg(frame,"bgr8"))
    rate.sleep()
    
    
if __name__ == '__main__':
  rospy.init_node('video', anonymous=True)
  try:
    video_play()
  except rospy.ROSInterruptException:
    pass
