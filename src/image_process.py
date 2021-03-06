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
global num, trial_num, data_set
num=0
trial_num=0
data_set="test"
class image_converter:

  def __init__(self):
    global num, trial_num, data_set
    trial_num = rospy.get_param('/trial_num')
    data_set = rospy.get_param('/data_set')
    rospy.loginfo("Started trial number: "+str(trial_num))
    self.image_pub = rospy.Publisher("/turtle_follow/output/image_raw",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/turtle_follow/usb_cam_node/image_raw",Image,self.callback)

    self.drive_sub= rospy.Subscriber("/turtle_follow/output/drive_out",String,self.callback1)
    print("done.")


  def callback1(self,data):
    global current
    current = data.data

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    resized_image = cv2.resize(gray_image, (100, 100))
    resized_image = resized_image[50:100, 0:100]
    resized_image = cv2.resize(resized_image, (50, 50))

    cv2.waitKey(3)
    #rospy.loginfo(current)
    try:
      global num, trial_num

      rospy.loginfo("Published image.")
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(resized_image, "mono8"))
      if current != 'x':
          num+=1
          pub_string = "/home/rrc/data/set/"+str(data_set)+"/"+str(current)+"/trial_"+str(trial_num)+"_img_"+str(num)+"_"+str(current)+".png"
          rospy.loginfo("Wrote image:"+pub_string)
          cv2.imwrite( pub_string , resized_image );
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  rospy.loginfo("Started image_process")
  ic = image_converter()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    print("number last was "+num)
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
