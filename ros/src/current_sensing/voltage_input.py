#! /usr/bin/env python

import sys
import time
import traceback
import rospy


from Phidget22.Devices.VoltageInput import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *
from std_msgs.msg import Float64


global Voltage_value
Voltage_value = [0]

try:
    from PhidgetHelperFunctions import *
except ImportError:
    sys.stderr.write("\nCould not find PhidgetHelperFunctions. Either add PhdiegtHelperFunctions.py to your project folder "
                      "or remove the import from your project.")
    sys.stderr.write("\nPress ENTER to end program.")
    readin = sys.stdin.readline()
    sys.exit()

"""
* Configures the device's DataInterval and ChangeTrigger.
* Displays info about the attached Phidget channel.
* Fired when a Phidget channel with onAttachHandler registered attaches
*
* @param self The Phidget channel that fired the attach event
"""
class VoltageReader():
    
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
            #If you are unsure how to use more than one Phidget channel with this event, we recommend going to
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
        Voltage_value[0] = voltage
    
    def onSensorChangeHandler(self, ph, sensorValue, sensorUnit):

        print("[Sensor Event] -> Sensor Value: " + str(sensorValue) + sensorUnit.symbol)

    def __init__(self):
        self.voltage_publisher = rospy.Publisher('CurrentSensor/battery_voltage_draw', Float64, queue_size=10)
    	self._hz = 10
        ch1 = VoltageInput()
           
        channelInfo = ChannelInfo()
        channelInfo.deviceSerialNumber = 539331
        channelInfo.hubPort = 1
        channelInfo.isHubPortDevice = 0
        channelInfo.channel = 0
           
        ch1.setDeviceSerialNumber(channelInfo.deviceSerialNumber)
        ch1.setHubPort(channelInfo.hubPort)
        ch1.setIsHubPortDevice(channelInfo.isHubPortDevice)
        ch1.setChannel(channelInfo.channel)   
            
        ch1.setOnAttachHandler(self.onAttachHandler)
        ch1.setOnDetachHandler(self.onDetachHandler)
        ch1.setOnErrorHandler(self.onErrorHandler)
        ch1.setOnVoltageChangeHandler(self.onVoltageChangeHandler)
        ch1.setOnSensorChangeHandler(self.onSensorChangeHandler)
            
        print("\nOpening and Waiting for Attachment...")
        
        try:
            ch1.openWaitForAttachment(5000)
        except PhidgetException as e:
            PrintOpenErrorMessage(e, ch1)
            raise EndProgramSignal("Program Terminated: Open Failed")
    
    def run(self):
    	rate = rospy.Rate(self._hz)
        while not rospy.is_shutdown():
    	    self._publish()
            rate.sleep()
    
    def _publish(self):            
        self.voltage_publisher.publish(Voltage_value[0])
                    
def main():
    rospy.init_node('battery_voltage')
    rospy.loginfo('Battery Voltage Node Started')
    run = VoltageReader()
    run.run()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
