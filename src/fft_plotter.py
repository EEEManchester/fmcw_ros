#! /usr/bin/env python3

# DATE: 05/06/2019
# AUTHOR: J.ROE [ORCA PROJECT]

import rospy
from std_msgs.msg import Float32MultiArray
#import numpy as np 
from matplotlib import pyplot as plt 

def callback(data):
 global FFTdata
 FFTdata = data.data 

rospy.init_node('fft_plotter') #initialise node
rate = rospy.Rate(5) #run at 5 Hz
rospy.Subscriber("fmcw/fft", Float32MultiArray, callback) #subscribe to radar FFT
global FFTdata #define FFTdata as global variable
FFTdata = [0,0] #placeholder data for FFtdata
fig = plt.gcf() #define the figure
fig.show() #display the figure window
fig.canvas.draw() #draw the line

#main loop
while not rospy.is_shutdown():
 plt.clf() #clear the plot
 plt.plot(FFTdata) #plot the data as a line
 plt.ylim([0, 0.5]) #lock the y axis at 0-0.5
 plt.pause(0.01) #pause for matplotlib
 fig.canvas.draw() #draw the line
 rate.sleep() #sleep for 0.2s @ 5hz
