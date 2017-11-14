#!/usr/bin/env python

import rospy, roslib, sys, cv2, os,rosbag
from cv_bridge import CvBridge, CvBridgeError
from rospkg import RosPack
rp = RosPack()
path = rp.get_path('air_hockey')

cvb = CvBridge()
fourcc = cv2.cv.CV_FOURCC('D','I','V','X')
out = cv2.VideoWriter(path+"/bagfiles/Air_Hockey_PS3.avi",fourcc,60.30,(640,480))

bag = rosbag.Bag(path+"/bagfiles/Air_Hockey_PS3.bag")
print "CREATING VIDEO FROM BAG FILE INITIATED..."
for topic, msg, t in bag.read_messages():
    if topic == "/usb_cam/image_raw":
        frame = cvb.imgmsg_to_cv2(msg,"bgr8")
        out.write(frame)
print "...VIDEO CREATED"
bag.close()
out.release()
