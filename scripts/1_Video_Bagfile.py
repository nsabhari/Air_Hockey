#!/usr/bin/env python

import rospy, cv2, rosbag
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from rospkg import RosPack
rp = RosPack()
path = rp.get_path('air_hockey')

cvb = CvBridge()

bag = rosbag.Bag(path+"/bagfiles/Air_Hockey_PS3.bag",'w')
cap = cv2.VideoCapture(path+"/bagfiles/Air_Hockey_PS3.avi")
print "CREATING BAG FROM VIDEO FILE INITIATED..."

while True:
    ret, frame = cap.read()
    if ret == False:
        break
    msg = cvb.cv2_to_imgmsg(np.array(frame),"bgr8")
    bag.write("/usb_cam/image_raw",msg)

print "...VIDEO CREATED"
bag.close()

