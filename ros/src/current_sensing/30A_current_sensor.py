#! /usr/bin/env python

import sys
import time
import traceback
import rospy


from Phidget22.Devices.CurrentInput import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *
from std_msgs.msg import Float64
from playsound import playsound

global current_value
current_value = [0]

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
            * DataInterval defines the minimum time between CurrentChange events.
            * DataInterval can be set to any value from MinDataInterval to MaxDataInterval.
            """
            print("\n\tSetting DataInterval to 100ms")
            ph.setDataInterval(100)
    
            """
            * Set the CurrentChangeTrigger inside of the attach handler to initialize the device with this value.
a           * CurrentChangeTrigger will affect the frequency of CurrentChange events, by limiting them to only occur when
            * the current changes by at least the value set.
            """
            print("\tSetting Current ChangeTrigger to 0.0")
            ph.setCurrentChangeTrigger(0.0)
            
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
    * Outputs the CurrentInput's most recently reported current.
    * Fired when a CurrentInput channel with onCurrentChangeHandler registered meets DataInterval and ChangeTrigger criteria
    *
    * @param self The CurrentInput channel that fired the CurrentChange event
    * @param current The reported current from the CurrentInput channel
    """
    def onCurrentChangeHandler(self, current):
        #print("[Current Event] -> Current: " + str(current))
        current_value[0] = current
    
    """
    * Prints descriptions of how events related to this class work
    """
    def PrintEventDescriptions():
    
        print("\n--------------------\n"
            "\n  | Current change events will call their associated function every time new current data is received from   the device.\n"
            "  | The rate of these events can be set by adjusting the DataInterval for the channel.\n"
            "  | Press ENTER once you have read this message.")
        readin = sys.stdin.readline(1)
        
        print("\n--------------------")
       
    """
    * Creates, configures, and opens a CurrentInput channel.
    * Displays Current events for 10 seconds
    * Closes out CurrentInput channel
    *
    * @return 0 if the program exits successfully, 1 if it exits with errors.
    """
    def __init__(self):
    	self.current_publisher = rospy.Publisher('CurrentSensor/battery_current_draw', Float64, queue_size=10)
    	self._hz = rospy.get_param('~hz' , 50)
    	try:
            """
            * Allocate a new Phidget Channel object
            """
            ch = CurrentInput()         
            
            channelInfo = ChannelInfo()
            channelInfo.deviceSerialNumber = 539331
            channelInfo.hubPort = 2
            channelInfo.isHubPortDevice = 0
            channelInfo.channel = 0
	    
            ch.setDeviceSerialNumber(channelInfo.deviceSerialNumber)
            ch.setHubPort(channelInfo.hubPort)
            ch.setIsHubPortDevice(channelInfo.isHubPortDevice)
            ch.setChannel(channelInfo.channel)  
             
            ch.setOnAttachHandler(onAttachHandler)
            ch.setOnDetachHandler(onDetachHandler)
            ch.setOnErrorHandler(onErrorHandler)
            ch.setOnCurrentChangeHandler(onCurrentChangeHandler)
            
            """
            * Open the channel with a timeout
            """
            
            print("\nOpening and Waiting for Attachment...")
            
            try:
                ch.openWaitForAttachment(5000)
            except PhidgetException as e:
                PrintOpenErrorMessage(e, ch)
                raise EndProgramSignal("Program Terminated: Open Failed")
        except:
            pass
    	
    def run(self):
    	rate = rospy.Rate(self._hz)
    	self._running = True
    	self._publish()
    
    def _publish(self):            
        while True:
            self.current_publisher.publish(float(current_value[0])*-1)
            if current_value[0] > 15 : playsound('/home/mars/Downloads/torture.wav')  #15 amps 
            time.sleep(0.1)   
    
            
def main():
    print("my main function is working")
    rospy.init_node('thirtyamp_battery_current')
    run = CurrentReader()
    run.run()
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
