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

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

#from __future__ import print_function
from time import sleep
from os import system
from sys import stdout

from uldaq import (get_daq_device_inventory, DaqDevice, AInScanFlag, ScanStatus,
                   ScanOption, create_float_buffer, InterfaceType, AiInputMode)

def talker_beam():
    pub = rospy.Publisher('chatter_beam', String, queue_size=10)
    rospy.init_node('talker_beam', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    global pre_index
    daq_device = None
    ai_device = None
    status = ScanStatus.IDLE

    range_index = 0
    interface_type = InterfaceType.ANY
    low_channel = 0
    high_channel = 3
    
    rate = 10000
    samples_per_channel = rate
    scan_options = ScanOption.CONTINUOUS
    flags = AInScanFlag.DEFAULT
    total_count = samples_per_channel * (high_channel - low_channel + 1)
    try:
        devices = get_daq_device_inventory(interface_type)
        number_of_devices = len(devices)
        if number_of_devices == 0:
            raise RuntimeError('Error: No DAQ devices found')

        print('Found', number_of_devices, 'DAQ device(s):')
        for i in range(number_of_devices):
            print('  [', i, '] ', devices[i].product_name, ' (',
                  devices[i].unique_id, ')', sep='')
            
        descriptor_index = int(0)
        if descriptor_index not in range(number_of_devices):
            raise RuntimeError('Error: Invalid descriptor index')
        daq_device = DaqDevice(devices[descriptor_index])
        ai_device = daq_device.get_ai_device()
        if ai_device is None:
            raise RuntimeError('Error: The DAQ device does not support analog '
                               'input')

        # Create the DAQ device from the descriptor at the specified index.
        daq_device = DaqDevice(devices[descriptor_index])
        ai_info = ai_device.get_info()
        if not ai_info.has_pacer():
            raise RuntimeError('\nError: The specified DAQ device does not '
                               'support hardware paced analog input')
        # Establish a connection to the DAQ device.
        descriptor = daq_device.get_descriptor()
        print('\nConnecting to', descriptor.dev_string, '- please wait...')
        # For Ethernet devices using a connection_code other than the default
        # value of zero, change the line below to enter the desired code.
        daq_device.connect(connection_code=0)

        # The default input mode is SINGLE_ENDED.
        input_mode = AiInputMode.DIFFERENTIAL
        # If SINGLE_ENDED input mode is not supported, set to DIFFERENTIAL.
        if ai_info.get_num_chans_by_mode(AiInputMode.SINGLE_ENDED) <= 0:
            input_mode = AiInputMode.DIFFERENTIAL

        # Get the number of channels and validate the high channel number.
        number_of_channels = ai_info.get_num_chans_by_mode(input_mode)
        if high_channel >= number_of_channels:
            high_channel = number_of_channels - 1
        channel_count = high_channel - low_channel + 1
        ranges = ai_info.get_ranges(input_mode)
        if range_index >= len(ranges):
            range_index = len(ranges) - 1
        data = create_float_buffer(channel_count, samples_per_channel)

        system('clear')

        # Start the acquisition.
        rate = ai_device.a_in_scan(low_channel, high_channel, input_mode,
                                   ranges[range_index], samples_per_channel,
                                   rate, scan_options, flags, data)
        
        try:
            
            while not rospy.is_shutdown():
                try:
                    status, transfer_status = ai_device.get_scan_status()

                    reset_cursor()
                    
                    index = transfer_status.current_index
                    #*******************
                    curr_index = transfer_status.current_index
                    if curr_index == -1:
                        sleep(0.5)
                        continue
                    pre_index = int(curr_index - (rate/10)*4)
                    
                    if 0 < pre_index:
                        #print('currentIndex - pre_index = ',index-pre_index, '\n')
                        data_r = data[pre_index :index]
                    elif 0 > pre_index:
                        #print('currentIndex - pre_index = ',total_count+index-pre_index, '\n')
                        data_r = data[pre_index+total_count:total_count]
                        #data_r.append(data[0:index])
                        data_r = data_r + data[0:index]
                    ch1 = data_r[::4]
                    ch2 = data_r[1::4]
                    ch3 = data_r[2::4]
                    ch4 = data_r[3::4]
                    pre_index = curr_index
                    ch1_max = max(ch1)
                    result = "Max = " + str(ch1_max)
                    rospy.loginfo(result)
                    pub.publish(result)
                    rate.sleep()
                    
                except (ValueError, NameError, SyntaxError):
                    break
        
        except KeyboardInterrupt:
            pass
    except RuntimeError as error:
        print('\n', error)

    finally:
        if daq_device:
            # Stop the acquisition if it is still running.
            if status == ScanStatus.RUNNING:
                ai_device.scan_stop()
            if daq_device.is_connected():
                daq_device.disconnect()
            daq_device.release()

if __name__ == '__main__':
    try:
        pre_index = 0
        talker_beam()
    except rospy.ROSInterruptException:
        pass
