#! /usr/bin/env python

import sys
import time
import traceback
import rospy

from Phidget22.Devices.CurrentInput import *
from Phidget22.Devices.VoltageInput import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *
from std_msgs.msg import Float64

global current_value
global voltage_value
current_value = [0,0]
voltage_value = [0]

try:
    from PhidgetHelperFunctions import *
except ImportError:
    sys.stderr.write("\nCould not find PhidgetHelperFunctions. "
                      "or remove the import from your project.")
    sys.stderr.write("\nPress ENTER to end program.")
    readin = sys.stdin.readline()
    sys.exit()

class SensorReader():
    def onAttachHandler(self, ph):
        try:

            print("\nAttach Event:")

            """
            * Get device information and display it.
            """
            channelClassName = ph.getChannelClassName()
            serialNumber = ph.getDeviceSerialNumber()
            channel = ph.getChannel()
            if(ph.getDeviceClass() == DeviceClass.PHIDCLASS_VINT):
                hubPort = ph.getHubPort()
                print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) +
                    "\n\t-> Hub Port: " + str(hubPort) + "\n\t-> Channel:  " + str(channel) + "\n")
            else:
                print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) +
                        "\n\t-> Channel:  " + str(channel) + "\n")


            print("\n\tSetting DataInterval to 1000ms")
            ph.setDataInterval(1000)

            print("\tSetting Voltage ChangeTrigger to 0.0")
            ph.setVoltageChangeTrigger(0.0)

            if(ph.getChannelSubclass() == ChannelSubclass.PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT):
                print("\tSetting Voltage SensorType")
                ph.setSensorType(VoltageSensorType.SENSOR_TYPE_VOLTAGE)

        except PhidgetException as e:
            print("\nError in Attach Event:")
            DisplayError(e)
            traceback.print_exc()
            return
    def onDetachHandler(self, ph):

        try:
            #If you are unsure how to use more than one Phidget channel with this event, we recommen$
            #www.phidgets.com/docs/Using_Multiple_Phidgets for information

            print("\nDetach Event:")

            serialNumber = ph.getDeviceSerialNumber()
            channelClass = ph.getChannelClassName()
            channel = ph.getChannel()

            deviceClass = ph.getDeviceClass()
            if (deviceClass != DeviceClass.PHIDCLASS_VINT):
                print("\n\t-> Channel Class: " + channelClass + "\n\t-> Serial Number: " + str(serialNumber) +
                      "\n\t-> Channel:  " + str(channel) + "\n")
            else:            
                hubPort = ph.getHubPort()
                print("\n\t-> Channel Class: " + channelClass + "\n\t-> Serial Number: " + str(serialNumber) +
                      "\n\t-> Hub Port: " + str(hubPort) + "\n\t-> Channel:  " + str(channel) + "\n")

        except PhidgetException as e:
            print("\nError in Detach Event:")
            DisplayError(e)
            traceback.print_exc()
            return

    def onErrorHandler(self, ph, errorCode, errorString):
	sys.stderr.write("[Phidget Error Event] -> " + errorString + " (" + str(errorCode) + ")\n")

    def onVoltageChangeHandler(self, ph, voltage):
	print("[Voltage Event] -> Voltage: " + str(voltage))
	voltage_value[0] = voltage
    def onCurrentChangeHandler_m(self, ph, current):
	current_value[0] = current
    def onCurrentChangeHandler_b(self, ph, current):	
	current_value[1] = current

    def __init__(self):
	self.motor_current_publisher = rospy.Publisher('phidgetsensor/motor_current_draw', Float64, queue_size=10)
	self.voltage_publisher = rospy.Publisher('phidgetsensor/battery_voltage' , Float64, queue_size=10)
	self.battery_current_publisher = rospy.Publisher('phidgetsensor/battery_current_draw', Float64, queue_size=10)

	self._hz = 10
	
	#Channel 1 will be the motor_current_draw
	ch1 = CurrentInput()

	channelInfo = ChannelInfo()
	channelInfo.deviceSerialNumber = 539331
	channelInfo.hubPort = 2
	channelInfo.isHubPortDevice = 0
	channelInfo.channel = 0

	ch1.setDeviceSerialNumber(channelInfo.deviceSerialNumber)
        ch1.setHubPort(channelInfo.hubPort)
        ch1.setIsHubPortDevice(channelInfo.isHubPortDevice)
        ch1.setChannel(channelInfo.channel) 

        ch1.setOnAttachHandler(self.onAttachHandler)
        ch1.setOnDetachHandler(self.onDetachHandler)
        ch1.setOnErrorHandler(self.onErrorHandler)
        ch1.setOnCurrentChangeHandler(self.onCurrentChangeHandler_m)
	
	try: 
	    ch1.openWaitForAttachment(5000)
	except PhidgetException as e:
	    PrintOpenErrorMessage(e, ch1)
	    raise EndProgramSignal("Program Terminated: Open ch1 failed")
	#channel 2 will be the battery voltage
	ch2 = VoltageInput()

        channelInfo = ChannelInfo()
        channelInfo.deviceSerialNumber = 539331
        channelInfo.hubPort = 1
        channelInfo.isHubPortDevice = 0
        channelInfo.channel = 0

        ch2.setDeviceSerialNumber(channelInfo.deviceSerialNumber)
        ch2.setHubPort(channelInfo.hubPort)
        ch2.setIsHubPortDevice(channelInfo.isHubPortDevice)
        ch2.setChannel(channelInfo.channel) 

        ch2.setOnAttachHandler(self.onAttachHandler)
        ch2.setOnDetachHandler(self.onDetachHandler)
        ch2.setOnErrorHandler(self.onErrorHandler)
        ch2.setOnVoltageChangeHandler(self.onVoltageChangeHandler)

        try: 
            ch2.openWaitForAttachment(5000)
        except PhidgetException as e:
            PrintOpenErrorMessage(e, ch2)
            raise EndProgramSignal("Program Terminated: Open ch2 failed")

	#channel 3 will be the battery current
	ch3 = CurrentInput()

        channelInfo = ChannelInfo()
        channelInfo.deviceSerialNumber = 539331
        channelInfo.hubPort = 0
        channelInfo.isHubPortDevice = 0
        channelInfo.channel = 0

        ch3.setDeviceSerialNumber(channelInfo.deviceSerialNumber)
        ch3.setHubPort(channelInfo.hubPort)
        ch3.setIsHubPortDevice(channelInfo.isHubPortDevice)
        ch3.setChannel(channelInfo.channel) 

        ch3.setOnAttachHandler(self.onAttachHandler)
        ch3.setOnDetachHandler(self.onDetachHandler)
        ch3.setOnErrorHandler(self.onErrorHandler)
        ch3.setOnCurrentChangeHandler(self.onCurrentChangeHandler_b)
	
        try: 
            ch3.openWaitForAttachment(5000)
        except PhidgetException as e:
            PrintOpenErrorMessage(e, ch3)
            raise EndProgramSignal("Program Terminated: Open ch3 failed")

    def run(self):
	rate = rospy.Rate(self._hz)
	while not rospy.is_shutdown():
	    self._publish()
	    rate.sleep()

    def _publish(self):
	self.motor_current_publisher.publish(current_value[0])
	self.voltage_publisher.publish(voltage_value[0])
	self.battery_current_publisher.publish(current_value[1])

def main():
    rospy.init_node("Phidgets Sensors")
    rospy.loginfo("Sensors Node Started")
    run = SensorReader()
    run.run()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
