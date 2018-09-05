#!/usr/bin/env python

'''
Odom.py
CDT Born and CDT Zahin

This node subscribes to /odom and to the ticks and heading topics.
It then displays a graph that tracks the location of the robot
using both UTM gps tracking and odometry.

This node was run remotely.
'''

import rospy, time, math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
import matplotlib.pyplot as pp
import os

global x,y
global points
global trial_num
points = []
global current
current='x'
global to_plot
to_plot=False

def drive_callback(data):
    global current
    current=data.data

def plot_callback(data):
    global to_plot
    if data.data == 'true':
        to_plot=True
    else:
        to_plot=False

def pose_callback(data):
    global x,y
    x= data.pose.position.x
    y= data.pose.position.y

def plot():
    #rotary code here
    global x, y, current

    global points

    if to_plot:
        points.append([x,y])
        pp.plot(x,y, 'bo')
        pp.draw()
        pp.pause(0.000001)



    #f.write(str(seconds)+","+str(x_coord)+","+str(y_coord))

def shutdown():
    global points
    write= ""
    for p in points:
        write+= str(p[0])+','+str(p[1])+";"
    with open('/home/wborn/auto_trials/trial_'+str(rospy.get_param('/trial_num')), 'wb') as fp:
        fp.write(write)
        fp.close()
    pass


def init():
    rospy.init_node('plot',anonymous=True)
    pp.show(block=False)

    global x,y
    x=0
    y=0

    raw = []
    for filename in os.listdir("/home/wborn/course"):
        fp = open("/home/wborn/course/"+filename,'r')
        raw.append(fp.read().split(';'))
        fp.close()


    points = []
    for i in raw:
        temp=[]
        for j in i:
            if j.split(',') != ['']:
                temp.append(j.split(','))
        points.append(temp)
    points=[x for x in points if x != ['']]

    for j in range(len(points)):
        for i in range(len(points[j])-1):
            pp.plot([float(points[j][i][0]),float(points[j][i+1][0])],[float(points[j][i][1]),float(points[j][i+1][1])], '-',color='black',linewidth=7.0)
        #pp.draw()
        #pp.pause(0.000001)


    rospy.on_shutdown(shutdown)

    rospy.Subscriber("/turtle_follow/output/drive_out",String,drive_callback)
    rospy.Subscriber("/turtle_follow/output/plot",String,plot_callback)
    rospy.Subscriber("/vrpn_client_node/RigidBody1/pose", PoseStamped, pose_callback)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        global seconds
        seconds = rospy.get_time()
        plot()
        r.sleep()

init()
