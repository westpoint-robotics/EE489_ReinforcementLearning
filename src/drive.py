#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String,Float32

global buttons, axes, state, auto, fuzzy
buttons = [0,0,0,0,0,0,0,0,0,0,0]
axes = [0,0,0,0,0,0,0,0]
state = 0;
fuzzy =0.0
#data = []

global first_time


global last_fuzzy, last_turn, last_time
last_fuzzy=0
last_turn=0
last_time=0

import numpy as np
import matplotlib.pyplot as pp

current='x'
auto='s'

def callback(data):
    global buttons,axes
    buttons = data.buttons
    axes = data.axes

def auto_callback(data):
    global auto
    auto = data.data

def fuzzy_callback(data):
    global fuzzy
    fuzzy = data.data


class GoForward():

    def drive(self):
        global state,axes
        if axes[7]==1.0:
            state=0
        elif axes[7]==-1.0:
            state=2
        elif axes[6]==1.0:
            state=3
        elif axes[6]==-1.0:
            state=1

        if state==0:
            self.free_drive()
        elif state==1:
            self.trials()
        elif state==2:
            self.autonomous()
        elif state==3:
            self.fuzzy()

    def trials(self):
        self.plot_pub.publish('true')
        # let's go forward at 0.2 m/s
        # subscribe to joy nod
        global state
        rospy.loginfo(buttons[0])
        if buttons[0] == 1:
            state=0
            current = 'x'
            rospy.loginfo("Stop")
            self.move_cmd.linear.x = 0
            # turn at 0 radians/s
            self.move_cmd.angular.z = 0
        elif buttons[1] == 1:
            #data.append('r')
            current = 'r'
            rospy.loginfo("right")
            self.move_cmd.linear.x = 0.15
            # turn at -1 radians/s
            self.move_cmd.angular.z = -1.0
        elif buttons[2] == 1:
            #data.append('l')
            current = 'l'
            rospy.loginfo("left")
            self.move_cmd.linear.x = 0.15
            # turn at 1 radians/s
            self.move_cmd.angular.z = 1.0
        elif buttons[3] == 1:
             state=0
             current = 'x'
             rospy.loginfo("Stop")
             self.move_cmd.linear.x = 0
             # turn at 0 radians/s
             self.move_cmd.angular.z = 0
        else:
            #data.append('s')
            current = 's'
            rospy.loginfo("Straight")
            self.move_cmd.linear.x = .15
            # let's turn at 0 radians/s
            self.move_cmd.angular.z = 0
        # publish the velocity
        self.cmd_vel.publish(self.move_cmd)
        self.drive_pub.publish(current)
        # wait for 0.1 seconds (10 HZ) and publish again


    def free_drive(self):
        self.plot_pub.publish('false')
        global buttons,axes,state
        current = 'x'
        if buttons[3] == 1:
             state=1
        self.move_cmd.angular.z = axes[3] * 2
        self.move_cmd.linear.x = axes[1] / 2.5

        self.cmd_vel.publish(self.move_cmd)
        self.drive_pub.publish('x')

    def autonomous(self):
        turn = 0
        self.plot_pub.publish('true')
        rospy.Subscriber("/turtle_follow/output/drive_out", String, auto_callback)
        global auto,last_turn,last_time
        rospy.loginfo(auto)
        if auto == 's':
            turn=0
        elif auto == 'r':
            turn=-1
        elif auto == 'l':
            turn= 1
        else:
            turn=0
        global first_time
        time=rospy.get_rostime().secs-first_time
        pp.plot([last_time,time],[last_turn,turn],'b-')
        pp.draw()
        pp.pause(0.000001)
        last_turn=turn
        last_time=time
        self.move_cmd.linear.x = 0.15
        self.move_cmd.angular.z = turn
        self.cmd_vel.publish(self.move_cmd)

    def fuzzy(self):
        self.plot_pub.publish('true')
        rospy.loginfo(fuzzy)
        rospy.Subscriber("/turtle_follow/output/fuzzy", Float32, fuzzy_callback)
        global fuzzy, last_fuzzy, last_time
        self.move_cmd.linear.x = .15
        self.move_cmd.angular.z = -1*fuzzy
        global first_time
        time=rospy.get_rostime().secs-first_time
        rospy.loginfo("time was "+str(time))
        pp.plot([last_time,time],[last_fuzzy,fuzzy],'r-')
        pp.draw()
        pp.pause(0.000001)
        last_fuzzy=fuzzy
        last_time=time
        self.cmd_vel.publish(self.move_cmd)

    def __init__(self):
        # initiliaze
        rospy.init_node('GoForward', anonymous=False)
        self.drive_pub = rospy.Publisher("/turtle_follow/output/drive_out",String)
        self.plot_pub = rospy.Publisher("/turtle_follow/output/plot",String)

        # tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")


        # What function to call when you ctrl + c
        rospy.on_shutdown(self.shutdown)

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)




        #TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 10 HZ
        r = rospy.Rate(60);

        # Twist is a datatype for velocity
        self.move_cmd = Twist()

        rospy.Subscriber("/turtle_follow/joy", Joy, callback)

        global current
        # as long as you haven't ctrl + c keeping doing...
        global first_time
        first_time=rospy.get_rostime().secs
        while not rospy.is_shutdown():
            self.drive()
            r.sleep()



    def shutdown(self):
        #print(data)
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        self.drive_pub.publish("x")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)

if __name__ == '__main__':
    #try:
    GoForward()
    #except:
        #rospy.loginfo("GoForward node terminated.")
