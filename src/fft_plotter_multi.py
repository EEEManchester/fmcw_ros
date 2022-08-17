#! /usr/bin/env python

# DATE: 05/06/2019
# AUTHOR: J.ROE [ORCA PROJECT]

import rospy
from std_msgs.msg import Float32MultiArray, String
import numpy as np 
from matplotlib import pyplot as plt

from scipy.interpolate import spline
from scipy.signal import find_peaks

def callback(data):
 global FFTdata
 global FFTsmooth
 global peaks
 global peaksy
 FFTdata = data.data[40:100]
 peaks, _ = find_peaks(FFTdata, prominence=1000)
 peaksy = list( FFTdata[i] for i in peaks)
 xnew = np.linspace(0,len(FFTdata),300) #300 represents number of points to make between T.min and T.max
 #FFTsmooth = spline(range(len(FFTdata)),FFTdata,xnew)
 #peaks = map(conv, peaks)
 print peaks
 print len(peaksy)
 
def conv(data):
    return data * 6

def trigcallback(data):
 global trigger
 trigger = data.data

def epsilonR(_max,_mag):
    n1 = 1.00027
    gamma = _mag/_max
    rootGamma = np.sqrt(gamma)
    unityMinusRootGamma=1-rootGamma
    unityPlusRootGamma=1+rootGamma
    n3 = unityPlusRootGamma/unityMinusRootGamma
    n2=n1*n3
    return n2*n2


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

rospy.init_node('fft_plotter_multi') #initialise node
rate = rospy.Rate(5) #run at 5 Hz
rospy.Subscriber("fmcw/fft", Float32MultiArray, callback) #subscribe to radar FFT
rospy.Subscriber("fmcw/trig", String, trigcallback) #subscribe to trigger
global FFTdata #define FFTdata as global variable
global FFTsmooth
global peaks
global peaksy
global trigger

peaks = []
peaksy = []
FFTdata = [0,1] #placeholder data for FFtdata
FFTsmooth = [0,1]
trigger = ''
fig = plt.gcf() #define the figure
fig.show() #display the figure window
fig.canvas.draw() #draw the line
datas = []
names = []

#main loop
while not rospy.is_shutdown():
 if not trigger == '':
  datas.append(FFTdata)
  names.append(trigger)
  trigger = ''
 plt.clf() #clear the plot
 print len(np.linspace(0,300,50))
 print len(FFTdata)
 #plt.plot(np.linspace(0,300,50), FFTdata, label='live')
 plt.plot(FFTdata, label='live',lw=3)
 print len(peaks)
 print len(peaksy)
 print "-"
 if len(peaks) > 0:
  print peaksy[0]
  plt.plot(peaks, peaksy ,'bo')
 # for p in peaks:
#      plt.text(p, FFTdata[p], epsilonR(peaksy[0],FFTdata[p]), fontsize=12)
 n = 0
 for fft in datas:
  plt.plot(fft, label='save: '+names[n],lw=2) #plot the data as a line
  n = n + 1
 plt.legend(prop={'size': 30})
 plt.grid()
 plt.yscale("log", basey=10)
 plt.ylim([10000, 10000000]) #lock the y axis at 0-0.5
 #plt.ylim([0, 100000]) #lock the y axis at 0-0.5
 plt.pause(0.01) #pause for matplotlib
 fig.canvas.draw() #draw the line
 rate.sleep() #sleep for 0.2s @ 5hz
