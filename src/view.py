#!/usr/bin/python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

current="x"
num=0
class image_converter:

  def __init__(self):
    global num

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    self.drive_sub= rospy.Subscriber("/output/drive_out",String,self.callback1)
    print("done.")


  def callback1(self,data):
    global current
    current = data

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    resized_image = cv2.resize(cv_image, (100, 100))

    clahe = cv2.createCLAHE(clipLimit=3., tileGridSize=(8,8))

    lab = cv2.cvtColor(resized_image, cv2.COLOR_BGR2LAB)  # convert from BGR to LAB color space
    l, a, b = cv2.split(lab)  # split on 3 different channels

    l2 = clahe.apply(l)  # apply CLAHE to the L-channel

    lab = cv2.merge((l2,a,b))  # merge channels
    img2 = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    #gray_image = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    #gray_image = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)

    cv2.namedWindow('image')
    showimg = cv2.resize(img2, (500, 500))
    cv2.resizeWindow('image', 600,600)
    cv2.imshow('image', showimg)

    cv2.waitKey(3)
    rospy.loginfo(current)


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    print("number last was "+num)
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)