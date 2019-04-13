#! /usr/bin/env python

import sys
import time
import traceback
import rospy


from Phidget22.Devices.VoltageInput import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *
from std_msgs.msg import String


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
    global onAttachHandler
    global onDetachHandler
    global onErrorHandler
    global onCurrentChangeHandler
    
    def onAttachHandler(self):
    
        ph = self
        try:
            #If you are unsure how to use more than one Phidget channel with this event, we recommend going to
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
                print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) +
                    "\n\t-> Hub Port: " + str(hubPort) + "\n\t-> Channel:  " + str(channel) + "\n")
            else:
                print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) +
                        "\n\t-> Channel:  " + str(channel) + "\n")
            
            """
            * Set the DataInterval inside of the attach handler to initialize the device with this value.
            * DataInterval defines the minimum time between VoltageChange events.
            * DataInterval can be set to any value from MinDataInterval to MaxDataInterval.
            """
            print("\n\tSetting DataInterval to 1000ms")
            ph.setDataInterval(1000)

            """
            * Set the VoltageChangeTrigger inside of the attach handler to initialize the device with this value.
            * VoltageChangeTrigger will affect the frequency of VoltageChange events, by limiting them to only occur when
            * the voltage changes by at least the value set.
            """
            print("\tSetting Voltage ChangeTrigger to 0.0")
            ph.setVoltageChangeTrigger(0.0)
            
            """
            * Set the SensorType inside of the attach handler to initialize the device with this value.
            * You can find the appropriate SensorType for your sensor in its User Guide and the VoltageInput API
            * SensorType will apply the appropriate calculations to the voltage reported by the device
            * to convert it to the sensor's units.
            * SensorType can only be set for Sensor Port voltage inputs (VINT Ports and Analog Input Ports)
            """
            if(ph.getChannelSubclass() == ChannelSubclass.PHIDCHSUBCLASS_VOLTAGEINPUT_SENSOR_PORT):
                print("\tSetting Voltage SensorType")
                ph.setSensorType(VoltageSensorType.SENSOR_TYPE_VOLTAGE)
            
        except PhidgetException as e:
            print("\nError in Attach Event:")
            DisplayError(e)
            traceback.print_exc()
            return

    """
    * Displays info about the detached Phidget channel.
    * Fired when a Phidget channel with onDetachHandler registered detaches
    *
    * @param self The Phidget channel that fired the attach event
    """
    def onDetachHandler(self):

        ph = self
        try:
            #If you are unsure how to use more than one Phidget channel with this event, we recommend going to
            #www.phidgets.com/docs/Using_Multiple_Phidgets for information
        
            print("\nDetach Event:")
            
            """
            * Get device information and display it.
            """
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


    """
    * Writes Phidget error info to stderr.
    * Fired when a Phidget channel with onErrorHandler registered encounters an error in the library
    *
    * @param self The Phidget channel that fired the attach event
    * @param errorCode the code associated with the error of enum type ph.ErrorEventCode
    * @param errorString string containing the description of the error fired
    """
    def onErrorHandler(self, errorCode, errorString):

        sys.stderr.write("[Phidget Error Event] -> " + errorString + " (" + str(errorCode) + ")\n")

    """
    * Outputs the VoltageInput's most recently reported voltage.
    * Fired when a VoltageInput channel with onVoltageChangeHandler registered meets DataInterval and ChangeTrigger criteria
    *
    * @param self The VoltageInput channel that fired the VoltageChange event
    * @param voltage The reported voltage from the VoltageInput channel
    """
    def onVoltageChangeHandler(self, voltage):

        #print("[Voltage Event] -> Voltage: " + str(voltage))
        Voltage_value[0] = voltage
    """
    * Outputs the VoltageInput's most recently reported sensor value.
    * Fired when a VoltageInput channel with onSensorChangeHandler registered meets DataInterval and ChangeTrigger criteria
    *
    * @param self The VoltageInput channel that fired the SensorChange event
    * @param sensorValue The reported sensor value from the VoltageInput channel
    """
    def onSensorChangeHandler(self, sensorValue, sensorUnit):

        print("[Sensor Event] -> Sensor Value: " + str(sensorValue) + sensorUnit.symbol)


    """
    * Prints descriptions of how events related to this class work
    """
    def PrintEventDescriptions():

        print("\n--------------------\n"
            "\n  | Voltage change events will call their associated function every time new voltage data is received from the device.\n"
            "  | The rate of these events can be set by adjusting the DataInterval for the channel.\n")
            
        print(
            "\n  | Sensor change events contain the most recent sensor value received from the device.\n"
            "  | Sensor change events will occur instead of Voltage change events if the SensorType is changed from the default.\n"
            "  | Press ENTER once you have read this message.")
        readin = sys.stdin.readline(1)
        
        print("\n--------------------")
       
    """
    * Creates, configures, and opens a VoltageInput channel.
    * Displays Voltage events for 10 seconds
    * Closes out VoltageInput channel
    *
    * @return 0 if the program exits successfully, 1 if it exits with errors.
    """
    def __init__(self):
        self.voltage_publisher = rospy.Publisher('CurrentSensor/battery_voltage_draw', String, queue_size=10)
    	self._hz = rospy.get_param('~hz' , 10)
        try:
            """
            * Allocate a new Phidget Channel object
            """
            ch1 = VoltageInput()
            
            """
            * Set matching parameters to specify which channel to open
            """
            
            channelInfo = ChannelInfo()
            channelInfo.deviceSerialNumber = 539331
            channelInfo.hubPort = 1
            channelInfo.isHubPortDevice = 1
            channelInfo.channel = 0
            
            ch1.setDeviceSerialNumber(channelInfo.deviceSerialNumber)
            ch1.setHubPort(channelInfo.hubPort)
            ch1.setIsHubPortDevice(channelInfo.isHubPortDevice)
            ch1.setChannel(channelInfo.channel)   
            
            if(channelInfo.netInfo.isRemote):
                ch1.setIsRemote(channelInfo.netInfo.isRemote)
                if(channelInfo.netInfo.serverDiscovery):
                    try:
                        Net.enableServerDiscovery(PhidgetServerType.PHIDGETSERVER_DEVICEREMOTE)
                    except PhidgetException as e:
                        PrintEnableServerDiscoveryErrorMessage(e)
                        raise EndProgramSignal("Program Terminated: EnableServerDiscovery Failed")
                else:
                    Net.addServer("Server", channelInfo.netInfo.hostname,
                        channelInfo.netInfo.port, channelInfo.netInfo.password, 0)
            

            ch1.setOnAttachHandler(onAttachHandler)
            ch1.setOnDetachHandler(onDetachHandler)
            ch1.setOnErrorHandler(onErrorHandler)
            ch1.setOnVoltageChangeHandler(onVoltageChangeHandler)
            ch1.setOnSensorChangeHandler(onSensorChangeHandler)
            
            """
            * Open the channel with a timeout
            """
            
            print("\nOpening and Waiting for Attachment...")
            
            try:
                ch1.openWaitForAttachment(5000)
            except PhidgetException as e:
                PrintOpenErrorMessage(e, ch1)
                raise EndProgramSignal("Program Terminated: Open Failed")
        except:
            pass
    def run(self):
    	rate = rospy.Rate(self._hz)
    	self._running = True
    	self._publish()
    
    def _publish(self):            
        while True:
            self.voltage_publisher.publish(str(Voltage_value[0]))
            time.sleep(0.1)   
                    
def main():
    print("my main function is working")
    rospy.init_node('battery_voltage')
    run = VoltageReader()
    run.run()
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
