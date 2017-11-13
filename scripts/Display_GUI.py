#!/usr/bin/env python

import sys, time
import numpy as np
import cv2, roslib, rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray  
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import *
from cv_bridge import CvBridge, CvBridgeError
from Template_AH import Ui_Form
VERBOSE=False

def set_pers():
    F = open("/home/sabhari/catkin_AH/src/air_hockey/scripts/AH_data.txt","r")
    ui.X_lt.setProperty("value",F.readline())
    ui.Y_lt.setProperty("value",F.readline())
    ui.X_rt.setProperty("value",F.readline())
    ui.Y_rt.setProperty("value",F.readline())
    ui.X_lb.setProperty("value",F.readline())
    ui.Y_lb.setProperty("value",F.readline())
    ui.X_rb.setProperty("value",F.readline())
    ui.Y_rb.setProperty("value",F.readline())
    F.close()
    
app = QApplication(sys.argv)
window = QDialog()
ui = Ui_Form()
ui.setupUi(window)
set_pers()

f_pers = np.zeros((300,600,3), np.uint8)
f_puck = np.zeros((300,600,3), np.uint8)
f_circle = np.zeros((300,600,3), np.uint8)
xy = [0,0]
class My_GUI:

    def __init__(self):        
        self.perspective = rospy.Subscriber("/air_hockey/perspective",Image, self.callback_perspective,  queue_size = 1)
        self.puck = rospy.Subscriber("/air_hockey/puck",Image, self.callback_puck,  queue_size = 1)
        self.circle = rospy.Subscriber("/air_hockey/circle",Image, self.callback_circle,  queue_size = 1)
        self.cms = rospy.Subscriber("/air_hockey/position",Int16MultiArray ,self.callback_cms)
        self.pub_pers = rospy.Publisher('/air_hockey/pers_co', Int16MultiArray, queue_size=1)
        self.pub_color = rospy.Publisher('/air_hockey/color', Int16MultiArray, queue_size=1)
        self.bridge = CvBridge()
        
    def callback_perspective(self, data):
        global f_pers
        f_pers = self.bridge.imgmsg_to_cv2(data,"rgb8")

    def callback_puck(self, data):
        global f_puck
        f_puck = self.bridge.imgmsg_to_cv2(data,"rgb8")

    def callback_circle(self, data):
        global f_circle
        f_circle = self.bridge.imgmsg_to_cv2(data,"rgb8")

    def callback_cms(self, data):
        global xy
        xy = np.int32([data.data[0],data.data[1]])
          
def display():
    global xy
    pers = np.asarray([ui.X_lt.value(),ui.Y_lt.value(),ui.X_rt.value(),ui.Y_rt.value(),ui.X_lb.value(),ui.Y_lb.value(),ui.X_rb.value(),ui.Y_rb.value()], dtype = np.int)
    F = open("/home/sabhari/catkin_AH/src/air_hockey/scripts/AH_data.txt","w")
    #F.seek(0)
    F.truncate()
    F.write(str(ui.X_lt.value())+"\n")
    F.write(str(ui.Y_lt.value())+"\n")
    F.write(str(ui.X_rt.value())+"\n")
    F.write(str(ui.Y_rt.value())+"\n")
    F.write(str(ui.X_lb.value())+"\n")
    F.write(str(ui.Y_lb.value())+"\n")
    F.write(str(ui.X_rb.value())+"\n")
    F.write(str(ui.Y_rb.value())+"\n")
    F.close()
    ic.pub_pers.publish(data = pers.tolist())
    color_range = np.asarray([ui.Hue_L.value(),ui.Sat_L.value(),ui.Val_L.value(),ui.Hue_h.value(),ui.Sat_H.value(),ui.Val_H.value(),ui.spinBox.value()], dtype = np.int)
    ic.pub_color.publish(data = color_range.tolist())
    ui.Frame_Perspective.setPixmap(QPixmap(QtGui.QImage(f_pers,f_pers.shape[1],f_pers.shape[0],f_pers.strides[0],QtGui.QImage.Format_RGB888)))
    ui.Frame_puck.setPixmap(QPixmap(QtGui.QImage(f_puck,f_puck.shape[1],f_puck.shape[0],f_puck.strides[0],QtGui.QImage.Format_RGB888)))
    ui.Frame_prediction.setPixmap(QPixmap(QtGui.QImage(f_circle,f_circle.shape[1],f_circle.shape[0],f_circle.strides[0],QtGui.QImage.Format_RGB888)))
    ui.X_cms.display(xy[0])
    ui.Y_cms.display(xy[1])
    window.show()
    cv2.waitKey(1)

if __name__ == '__main__':
    ic = My_GUI()
    rospy.init_node('AH_GUI', anonymous=True)
    while not rospy.is_shutdown():
        rate = rospy.Rate(33)
        display()
        rate.sleep()


        
