#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
import smbus
import time

#Program termination flag.
term = False

# for RPI version 1, use "bus = smbus.SMBus(0)"
bus = smbus.SMBus(1)

#commands and constants
COMS_UNLOCK = 1
COMS_LOCK = 0
STATUS_HIGH = 3
STATUS_LOW = 4

MAX_DEVICES = 20
FREQ = 100 # polling frequency in hertz

#variables
last_attempted_drop = 0
deviceArray = list()

def loadCallBack(data): 

    rospy.loginfo("loadCallBack")


def dropCallBack(data):

    rospy.loginfo(dropCallBack)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.\

    rospy.init_node('listener', anonymous=True)

    rospy.loginfo("BeaonListenNode Started");


    rospy.Subscriber('beacon_load', String, loadCallBack);
    rospy.Subscriber('beacon_drop', String, dropCallBack);

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()


def find_devices(bus):
    print("Searching for devices")
    
    global deviceArray
    
    for address in range(128):
        try:
            #Check for device at address
            bus.read_byte(deviceArray)
            print("Device " + str(int(device)) + " found " + str(deviceArray.count(address)) + " times in deviceArray")

            #append new device to the connected devices list and check if should have previously dropped
            
            #Check if address exist in current array
            if deviceArray.count(address)<1 : 
                deviceArray.append(address)
                print("Device added " + str(int(address)))
                
                time.sleep(3)
                writeData(device,COMS_LOCK) # lock on connection
            else : #if already exists
                print("Device " + str(int(device)) + " already connected")
            
        except: #if byte read fails
            pass
    return 1