#!/usr/bin/python
import numpy as np
import keras
from keras.models import Sequential
from keras.layers import Activation
from keras.layers.core import Dense, Flatten, Dropout
from keras.optimizers import Adam
from keras.metrics import categorical_crossentropy
from keras.preprocessing.image import ImageDataGenerator
from keras.preprocessing.image import img_to_array, load_img
import itertools
from cv_bridge import CvBridge, CvBridgeError
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image

global image
image = img=load_img("/home/rrc/model_trial/image_test.png")
image = img_to_array(image)

vgg16_model = keras.applications.vgg16.VGG16(include_top=False, input_shape=(50,50,3),classes=3,pooling='max')

#print(vgg16_model.summary())
global model
model = Sequential()
for layer in vgg16_model.layers:
    model.add(layer)

model.layers.pop()


for layer in model.layers:
    layer.trainable = False

model.add(Dense(3, activation='softmax'))

model.compile(Adam(lr=.0001),loss='categorical_crossentropy', metrics=['accuracy'])

model.load_weights("/home/rrc/model_trial/model.h5")

def callback(data):
  rospy.loginfo("recieved image")
  global image
  try:
    cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
  except CvBridgeError as e:
    print(e)
  image = cv_image


rospy.init_node('ac', anonymous=True)
drive_pub = rospy.Publisher("/turtle_follow/output/drive_out",String)
fuzzy_pub = rospy.Publisher("/turtle_follow/output/fuzzy",Float32)
image_sub = rospy.Subscriber("/turtle_follow/output/image_raw",Image,callback)

while not rospy.is_shutdown():
    rospy.loginfo("starting predictions")
    bin_predictions= model.predict_classes(image[None,:,:,:],batch_size=1)
    fuzzy_predictions= model.predict(image[None,:,:,:],batch_size=1)
    rospy.loginfo("done.")
    rospy.loginfo(bin_predictions)
    rospy.loginfo(fuzzy_predictions)
    if bin_predictions[0]==0:
        drive_pub.publish("l")
    elif bin_predictions[0]==1:
        drive_pub.publish("s")
    elif bin_predictions[0]==2:
        drive_pub.publish("r")
    fuzzy_pub.publish(fuzzy_predictions[0][2]-fuzzy_predictions[0][0])
