# fmcw
 EVK02401 FMCW radar driver for ROS

# Installation
 
 This repo is structured as a ROS package.  Navigate to your preferred workspace's src folder e.g. `~catkin_ws/src` or `husky_ws/src`, clone this repo using `git clone --branch main https://github.com/EEEManchester/fmcw_ros.git`.
 
Change to your workspace top folder `cd ..` and perform a `catkin_make`.

# Usage

Connect the EVK02401 via usb using the debug port.

By default any ROS nodes will search for the device on ```/dev/ttyUSB0```, however, this can be changed by passing parameters to the fmcw_radar node or by editing the fft.yaml file.

To run the FMCW node: `roslaunch fmcw fmcw.launch` or `rosrun fmcw fmcw_radar.py`

The reported amplitude topic (``/fmcw/amplitude``) is reported from a specific bin.  The choice of bin can be changed on the fly using the rosparam server and dynamic reconfigure variables,  either via the commandline `` or using `rosrun rqt_reconfigure rqt_reconfigure`.  Find out more about [dynamically reconfigurable variables here](http://wiki.ros.org/dynamic_reconfigure).

# Visualisation

 There is no provided distance data in this repo, however, an individual FFT bin amplitude is published under ``/fmcw/amplitude``.
 
 To check this is being published correctly, either use `rostopic echo /fmcw/amplitude` or `rosrun rqt_plot rqt_plot` to visualise the change in amplitude.

The entire FFT amplitudes are also published, and can be visualised using pre-existing code in this repo.

 to run the FTT visualier,
 first run the driver as above, then;

 `rosrun fmcw ftt_plotter.py`

 The fft visualiser can be run from any machine with access to the fmcw ros topic,
 it does not have to be run from the machine connected to the radar

# Troubleshooting

 package/launch file not found:-
 
 ensure your workspace is sourced if not automatic e.g. `source ~/catkin_ws/devel setup.bash`

 
 cannot access radar port (permission denied):-

 ensure your user has access to the device e.g. `sudo chown :[your_user] /dev/ttyUSB0`

 for a permanent solution add user to the dialout group `sudo adduser [your_user] dialout`
