#! /usr/bin/env python

import sys
import signal
import time
import traceback
import rospy

from playsound import playsound
from Phidget22.Devices.VoltageInput import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *
from std_msgs.msg import String



global current
current = [0]

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
class CurrentReader():
	
	global onAttachHandler
	global onDetachHandler
	global onErrorHandler
	global onVoltageChangeHandler
	global onSensorChangeHandler
	

	def onAttachHandler(self):
		ph = self
		try:
			#www.phidgets.com/docs/Using_Multiple_Phidgets for information

			print("\nAttach Event:")

			"""
			* Get device information and display it.
			"""
			channelClassName = ph.getChannelClassName()
			serialNumber = ph.getDeviceSerialNumber()
			channel = ph.getChannel()
			if(ph.getDeviceClass() == DeviceClass.PHIDCLASS_VINT):
				hubPort = ph.getHubPort()
				print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + 	str(serialNumber) +
					"\n\t-> Hub Port: " + str(hubPort) + "\n\t-> Channel:  " + str(channel) + "\n")
			else:
				print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + 	str(serialNumber) +
						"\n\t-> Channel:  " + str(channel) + "\n")

			"""
			* Set the DataInterval inside of the attach handler to initialize the device with this 	value.
			* DataInterval defines the minimum time between VoltageChange events.
			* DataInterval can be set to any value from MinDataInterval to MaxDataInterval.
			"""
			print("\n\tSetting DataInterval to 100ms")
			ph.setDataInterval(100)

			"""
			* Set the VoltageChangeTrigger inside of the attach handler to initialize the device 	with this value.
			* VoltageChangeTrigger will affect the frequency of VoltageChange events, by limiting 	them to only occur when
			* the voltage changes by at least the value set.
			"""
			print("\tSetting Voltage ChangeTrigger to 0.0")
			ph.setVoltageChangeTrigger(0.0)

			"""
			* Set the SensorType inside of the attach handler to initialize the device with this 	value.
			* You can find the appropriate SensorType for your sensor in its User Guide and the 	VoltageInput API
			* SensorType will apply the appropriate calculations to the voltage reported by the 	device
			* to convert it to the sensor's units.read
			* SensorType can only be set for Sensor Port voltage inputs (VINT Ports and Analog 	Input Ports)
			"""
			if(ph.getChannelSubclass() == ChannelSubclass.PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT):
				print("\tSetting Voltage SensorType")
			ph.setSensorType(VoltageSensorType.SENSOR_TYPE_VOLTAGE)
		except PhidgetException as e:
			print("\nError in Attach Event:")
			DisplayError(e)
			traceback.print_exc()
			return
	
	def onDetachHandler(self):
	    ph = self
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
	
	def onErrorHandler(self, errorCode, errorString):
	    sys.stderr.write("[Phidget Error Event] -> " + errorString + " (" + str(errorCode) + ")\n")
	
	def onVoltageChangeHandler(self, voltage):

   		current[0] = voltage * 40  
	
	def onSensorChangeHandler(self, sensorValue, sensorUnit):
	    print("[Sensor Event] -> Sensor Value: " + str(sensorValue) + sensorUnit.symbol)


	#Code added to start converting this into a ros node

	def __init__(self):
		self.current_publisher = rospy.Publisher('CurrentSensor/battery_current_draw', String, queue_size=10)
		self._hz = rospy.get_param('~hz' , 10)
	

	def run(self):
		rate = rospy.Rate(self._hz)
		self._running = True
		self._publish()

	def _publish(self):
		"""
		* Allocate a new Phidget Channel object
		"""

		try:
			try:
				ch = VoltageInput()
			except PhidgetException as e:
				sys.stderr.write("Runtime Error -> Creating VoltageInput: \n\t")
				DisplayError(e)
				raise
			except RuntimeError as e:
				sys.stderr.write("Runtime Error -> Creating VoltageInput: \n\t" + e)
				raise
			
			def signal_handler(signal, frame):
				print("exiting...")
				raise EndProgramSignal("Program Terminated")
			signal.signal(signal.SIGINT, signal_handler)
			
			channelInfo = ChannelInfo()
			channelInfo.deviceSerialNumber = 539331
			channelInfo.hubPort = 0
			channelInfo.isHubPortDevice = 1
			channelInfo.channel = 0
	
			ch.setDeviceSerialNumber(channelInfo.deviceSerialNumber)
			ch.setHubPort(channelInfo.hubPort)
			ch.setIsHubPortDevice(channelInfo.isHubPortDevice)
			ch.setChannel(channelInfo.channel)
				
			"""
			* Add event handlers before calling open so that no events are missed.
			"""	
			print(channelInfo.deviceSerialNumber)
			print(channelInfo.hubPort)
			ch.setOnAttachHandler(onAttachHandler)
			ch.setOnDetachHandler(onDetachHandler)
			ch.setOnErrorHandler(onErrorHandler)
			ch.setOnVoltageChangeHandler(onVoltageChangeHandler)
			ch.setOnSensorChangeHandler(onSensorChangeHandler)
			
			print("\nOpening and Waiting for Attachment...")
			try:
				ch.openWaitForAttachment(5000)
			except PhidgetException as e:
				PrintOpenErrorMessage(e, ch)
				raise EndProgramSign00al("Program Terminated: Open Failed")
			

			print("running...")
			rate = rospy.Rate(self._hz)
			while True:
				self.current_publisher.publish(str(current[0]))
				if current[0] > 14 : playsound('/home/mars/Downloads/torture.wav')
				rate.sleep()

		except PhidgetException as e:
			sys.stderr.write("\nExiting with error(s)...")
			DisplayError(e)
			traceback.print_exc()
			print("Cleaning up...")
			ch.setOnVoltageChangeHandler(None)
			ch.setOnSensorChangeHandler(None)
			ch.close()
			return 1
		except EndProgramSignal as e:
			print(e)
			print("Cleaning up...")
			ch.setOnVoltageChangeHandler(None)
			ch.setOnSensorChangeHandler(None)
			ch.close()
			return 1
		except KeyboardInterrupt:
			ch.setOnVoltageChangeHandler(None)
			ch.setOnSensorChangeHandler(None)
			ch.close()
			return 1
		
def main():
	print("my main function is working")
	rospy.init_node('battery_current')
	run = CurrentReader()
	run.run()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
