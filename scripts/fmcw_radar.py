#!/usr/bin/env python3

import rospy
import numpy as np
import serial

from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Float32
from fmcw.msg import ChannelAmplitudeStamped

from dynamic_reconfigure.server import Server
from fmcw.cfg import RadarSettingsConfig


import time

class Radar():

    def __init__(self):
        rospy.loginfo("Starting FMCW Radar Node")
        rospy.on_shutdown(self.safe_shutdown)


        # Get any initial parameters - not entirely necessary with the dynamic reconfigure functionality, but some variables are needed e.g. port, baud
        self.freqStart = self.frequency_check_GHz(rospy.get_param("~FREQUENCY_START", 24.0)) # Frequency start, default 24 GHz
        self.freqStop = self.frequency_check_GHz(rospy.get_param("~FREQUENCY_STOP", 25.5)) # Frequency stop, default 25.5 GHz
        self.freqPoints = rospy.get_param("~FREQUENCY_POINTS", 1501) # No. of bins for FFT?
        self.sweepTime = rospy.get_param("~SWEEP_TIME", 0.3) # Sweep time in seconds
        self.port = rospy.get_param("~port", "/dev/ttyUSB0") # USB serial port for comms
        self.baud = rospy.get_param("~baud", 115200) # Baud rate for comms
        self.sweepNumbers = rospy.get_param("~SWEEP_NUMBERS", 1)
        self.trigSource = rospy.get_param("~TRIGGER_SOURCE", "IMMEDIATE")
        self.hardware = rospy.get_param("~HARDWARE", "RS3400K")
        self.amplitudeBin = rospy.get_param("~AMPLITUDE_BIN", 9)
        self.enabled = rospy.get_param("~SWEEP_MEASURE", True)
        self.sensor_frame = rospy.get_param("~sensor_frame", "fmcw_antenna_link")

        self.initialised = False
        self.fft_data = []

        ###############################
        # Establish serial connection #
        ###############################
        rospy.loginfo("Establishing Serial connection on port %s", self.port)
        try:
            self.serialHandler = serial.Serial(port = self.port, baudrate = self.baud, timeout=2) #initialise serial connection with Nucleo
        except serial.SerialException:
            rospy.logerr("Unable to connect to device on port %s", self.port)

        if not self.serialHandler.isOpen():
          self.serialHandler.open()

        rospy.loginfo("Connection Established")

        self.serialHandler.reset_input_buffer()
        self.serialHandler.reset_output_buffer()
        ################################

        #########################
        # SET UP ROS COMPONENTS #
        #########################
        self.overhead = 1.7 # Additional time required for data transfer etc <= 1.6 secs
        self.rate = rospy.Rate(1/(self.sweepTime + self.overhead)) # Roughly 2 Hz


        self.fft_magnitude = Float32MultiArray()
        self.distance = Range()
        self.amplitude = ChannelAmplitudeStamped()
        self.phase = ChannelAmplitudeStamped()

        # Set up publishers for distance, full FFT and single channel amplitude
        self.fftPublisher = rospy.Publisher('fmcw/fft', Float32MultiArray, queue_size=10)
        self.distancePublisher = rospy.Publisher('fmcw/distance', Range, queue_size=10)
        self.ampltitudePublisher = rospy.Publisher('fmcw/amplitude', ChannelAmplitudeStamped, queue_size=10)
        self.phasePublisher = rospy.Publisher('fmcw/phase', ChannelAmplitudeStamped, queue_size=10)

        # Set up dynamic reconfigure server
        self.previousConfig = {}
        self.dynamServer = Server(RadarSettingsConfig, self.dynamHandler)
        ##########################

        #########################
        # INITIALISE RADAR UNIT #
        #########################

        # Set parameters - could probably make this neater or hook into the dynamic config instead
        self.set_radar_param("HARDWARE_SYSTEM",  self.hardware)
        self.set_radar_param("FREQUENCY_START",  self.freqStart)
        self.set_radar_param("FREQUENCY_STOP",   self.freqStop)
        self.set_radar_param("FREQUENCY_POINTS", self.freqPoints)
        self.set_radar_param("SWEEP_TIME",       self.sweepTime)
        self.set_radar_param("SWEEP_MEASURE",    self.enabled)
        self.set_radar_param("SWEEP_NUMBERS",    self.sweepNumbers)
        self.set_radar_param("TRIG_SOURCE",      self.trigSource)

        self.send_radar_cmd("INIT\r")
        self.initialised = True
        rospy.loginfo("Radar Initialised")

        #########################


        #############
        # MAIN LOOP #
        #############
        while not rospy.is_shutdown():
            if self.enabled:
                self.fft_data = self.read_radar_serial() # Get data
                self.process_radar_data(self.fft_data)
                self.publish_all_data()
            self.rate.sleep()
            
        #############


    def dynamHandler(self, config, level):
        # rospy.loginfo("Reconfigure Request Made")
        
        # If initialising parameters, create useful comparison dict
        if not self.previousConfig:
            self.previousConfig = dict.fromkeys(config)

        # Update variables
        self.freqStart = self.frequency_check_GHz(config["FREQUENCY_START"])
        self.freqStop = self.frequency_check_GHz(config["FREQUENCY_STOP"])
        self.freqPoints = config["FREQUENCY_POINTS"]
        self.sweepTime = config["SWEEP_TIME"]
        self.sweepNumbers = config["SWEEP_NUMBERS"]
        self.amplitudeBin = config["AMPLITUDE_BIN"]
        self.enabled = config["SWEEP_MEASURE"]

        # LOGGING AND UPDATING #
        # Detect which parameters have been changed
        parameter_changes = {i: config[i] for i in config if i in self.previousConfig and config[i] != self.previousConfig[i]}

        # Log any changes to parameters, and send to radar board
        if parameter_changes and self.initialised:
            for key, value in parameter_changes.items():
                if key != "groups" or key != "AMPLITUDE_BIN":
                    # self.set_radar_param(key, value)
                    rospy.loginfo("Parameter updated %s: %s", str(key), str(value))
            
        # Save current config
        self.previousConfig = config

        return config

    def frequency_check_GHz(self, frequency):
        # Check value in Hz by looking at magntiude - convert to GHz if Hz value given (e.g. 25e9 -> 25)
        if np.log10(frequency) > 9.0: # If in Hz (i.e. magnitude 1E9), return the value in GHz
            return (frequency / 1E9)
        else:
            return (frequency) # If the unit value, e.g. 24.0, return value


    def set_radar_param(self, param, value):
        temp_value = value
        
        # Check for bool in SWEEP_MEASURE, convert True/False to 'ON'/'OFF'
        if param == "SWEEP_MEASURE":
            temp_value = 'ON' if value == True else 'OFF'

        if param == "FREQUENCY_START" or param == "FREQUENCY_STOP":
            temp_value = "{:.2f}".format(value) + 'e9'

        formatted_param = param.replace("_", ":")
        formatted_value = str(temp_value)

        cmd_to_send = formatted_param + " " + formatted_value + "\r"
                
        self.send_radar_cmd(cmd_to_send)
        rospy.loginfo("Sending Cmd: %s", cmd_to_send)

    
    def read_radar_serial(self):
        self.send_radar_cmd("TRIG:ARM\r")

        self.serialHandler.reset_input_buffer() # Clear out rogue data in Rx buffer after getting TRIG reply

        data_payload = self.send_radar_cmd("TRAC:DATA ?\r")

        formatted_payload = data_payload.rstrip(bytes('OK\r\n', 'utf-8')).split(bytes('\r', 'utf-8')) # Might need to split further the bytes
        mapped_int_list = map(int, formatted_payload)
        return mapped_int_list

    def send_radar_cmd(self, payload_string):
        self.serialHandler.reset_output_buffer() # Clear out rogue data in Tx buffer

        self.serialHandler.write(bytes(payload_string, 'utf-8')) # Send command string
        
        while self.serialHandler.out_waiting > 0: # Wait for send buffer to be clear (i.e. payload sent)
            pass
        
        received_payload = self.serialHandler.read_until() # Wait for "OK" reply ending with '\n'
        
        return received_payload

    def process_radar_data(self, input_data):
        if input_data:
            try:
                serial_array = np.fromiter(input_data, dtype=np.int)
            except:
                return
            FFTdata = np.fft.fft(serial_array)
            FFTMag = np.abs(FFTdata)
            FFTPhase = np.angle(FFTdata)
        
            self.fft_magnitude = Float32MultiArray()

            self.fft_magnitude.data = FFTMag
            self.fft_magnitude.layout.data_offset = 0
            self.fft_magnitude.layout.dim.append(MultiArrayDimension())
            self.fft_magnitude.layout.dim[0].label = "amplitudes"
            self.fft_magnitude.layout.dim[0].size = len(FFTMag)
            self.fft_magnitude.layout.dim[0].stride = 1*len(FFTMag)
            
            # self.distance.range = 0
            
            # Extract amplitude (if index is in range)
            if self.amplitudeBin <= len(FFTdata):
                self.amplitude.amplitude = FFTMag[self.amplitudeBin - 1]
                self.phase.amplitude = FFTPhase[self.amplitudeBin - 1]
            else:
                self.amplitude.amplitude = 0.0
                self.phase.amplitude = 0.0
        else:
            return
        

    def publish_all_data(self):
        publish_time = rospy.Time.now()

        # FFT Magnitude
        self.fftPublisher.publish(self.fft_magnitude)

        # Distance/Range
        self.distance.header.stamp = publish_time
        self.distance.header.frame_id = self.sensor_frame
        self.distance.field_of_view = 0.2680 # radians - ~15.36 degrees
        self.distancePublisher.publish(self.distance)

        # Channel Amplitude
        self.amplitude.header.stamp = publish_time
        self.amplitude.header.frame_id = self.sensor_frame
        self.ampltitudePublisher.publish(self.amplitude)
        
        # Channel Phase
        self.phase.header = self.amplitude.header # Match amplitude
        self.phasePublisher.publish(self.phase)

    def safe_shutdown(self):
        # Turns out using self.serialHandler.close() is unnecessary
        rospy.loginfo("Closing Serial Port %s", self.port)





if __name__ == "__main__":
    rospy.init_node("fmcw_radar", anonymous=True)

    try:
        fmcw_radar = Radar()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
