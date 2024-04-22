#!/usr/bin/env python
# -*- coding: UTF-8 -*-

"""
Wrapper call demonstrated:        ai_device.a_in_scan()

Purpose:                          Performs a continuous scan of the range
                                  of A/D input channels

Demonstration:                    Displays the analog input data for the
                                  range of user-specified channels using
                                  the first supported range and input mode

Steps:
1.  Call get_daq_device_inventory() to get the list of available DAQ devices
2.  Create a DaqDevice object
3.  Call daq_device.get_ai_device() to get the ai_device object for the AI
    subsystem
4.  Verify the ai_device object is valid
5.  Call ai_device.get_info() to get the ai_info object for the AI subsystem
6.  Verify the analog input subsystem has a hardware pacer
7.  Call daq_device.connect() to establish a UL connection to the DAQ device
8.  Call ai_device.a_in_scan() to start the scan of A/D input channels
9.  Call ai_device.get_scan_status() to check the status of the background
    operation
10. Display the data for each channel
11. Call ai_device.scan_stop() to stop the background operation
12. Call daq_device.disconnect() and daq_device.release() before exiting the
    process.
"""
from __future__ import print_function
from time import sleep
from os import system
from sys import stdout
import rospy
from std_msgs.msg import String
import numpy as np
from uldaq import (get_daq_device_inventory, DaqDevice, AInScanFlag, ScanStatus,
                   ScanOption, create_float_buffer, InterfaceType, AiInputMode)
from beamfunc.beamfunc import *


def main():
    global pre_index
    """Analog input scan example."""
    pub = rospy.Publisher('daq', String, queue_size=10)
    #pub_c1 = rospy.Publisher('channel_1', String, queue_size=10)
    #pub_c2 = rospy.Publisher('channel_2', String, queue_size=10)
    #pub_c3 = rospy.Publisher('channel_3', String, queue_size=10)
    #pub_c4 = rospy.Publisher('channel_4', String, queue_size=10)

    rospy.init_node('talker_daq', anonymous=True)
    rate_sleep = rospy.Rate(1) # 10hz
    daq_device = None
    ai_device = None
    status = ScanStatus.IDLE

    range_index = 3
    interface_type = InterfaceType.ANY
    low_channel = 0
    high_channel = 3
    threshold = 0
    rate = 20000
    samples_per_channel = rate
    scan_options = ScanOption.CONTINUOUS
    flags = AInScanFlag.DEFAULT
    total_count = samples_per_channel * (high_channel - low_channel + 1)

    try:
        # Get descriptors for all of the available DAQ devices.
        devices = get_daq_device_inventory(interface_type)
        number_of_devices = len(devices)
        if number_of_devices == 0:
            raise RuntimeError('Error: No DAQ devices found')

        print('Found', number_of_devices, 'DAQ device(s):')
        for i in range(number_of_devices):
            print('  [', i, '] ', devices[i].product_name, ' (',
                  devices[i].unique_id, ')', sep='')

        # descriptor_index = input('\nPlease select a DAQ device, enter a number'
        #                          + ' between 0 and '
        #                          + str(number_of_devices - 1) + ': ')
        descriptor_index = int(0)
        if descriptor_index not in range(number_of_devices):
            raise RuntimeError('Error: Invalid descriptor index')

        # Create the DAQ device from the descriptor at the specified index.
        daq_device = DaqDevice(devices[descriptor_index])

        # Get the AiDevice object and verify that it is valid.
        ai_device = daq_device.get_ai_device()
        if ai_device is None:
            raise RuntimeError('Error: The DAQ device does not support analog '
                               'input')

        # Verify the specified device supports hardware pacing for analog input.
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

        # Get a list of supported ranges and validate the range index.
        ranges = ai_info.get_ranges(input_mode)
        if range_index >= len(ranges):
            range_index = len(ranges) - 1

        # Allocate a buffer to receive the data.
        data = create_float_buffer(channel_count, samples_per_channel)

        # print('\n', descriptor.dev_string, ' ready', sep='')
        # print('    Function demonstrated: ai_device.a_in_scan()')
        # print('    Channels: ', low_channel, '-', high_channel)
        # print('    Input mode: ', input_mode.name)
        # print('    Range: ', ranges[range_index].name)
        # print('    Samples per channel: ', samples_per_channel)
        # print('    Rate: ', rate, 'Hz')
        # print('    Scan options:', display_scan_options(scan_options))
        # try:
        #     input('\nHit ENTER to continue\n')
        # except (NameError, SyntaxError):
        #     pass

        system('clear')

        # Start the acquisition.
        rate = ai_device.a_in_scan(low_channel, high_channel, input_mode,
                                   ranges[range_index], samples_per_channel,
                                   rate, scan_options, flags, data)
        
        

        try:
            while not rospy.is_shutdown():
                try:
                    # Get the status of the background operation
                    status, transfer_status = ai_device.get_scan_status()

                    reset_cursor()
                    # print('Please enter CTRL + C to terminate the process\n')
                    # print('Active DAQ device: ', descriptor.dev_string, ' (',
                    #       descriptor.unique_id, ')\n', sep='')

                    # print('actual scan rate = ', '{:.6f}'.format(rate), 'Hz\n')

                    index = transfer_status.current_index
                    #*******************
                    curr_index = transfer_status.current_index
                    if curr_index == -1:
                        sleep(0.5)
                        continue
                    pre_index = int(curr_index - (rate/10)*4)
                    #print("curr_index = ",curr_index)
                    #print("pre_index  = ",pre_index)
                    #if curr_index > pre_index:
                    #    print('curr_index - pre_index = ',curr_index-pre_index)
                    #elif curr_index < pre_index:
                    #    print('curr_index - pre_index = ',40000-curr_index+pre_index)
                    #print("curr_index = ",curr_index)
                    
                    #*******************
                    # print('currentTotalCount = ',
                    #       transfer_status.current_total_count)
                    # print('currentScanCount = ',
                    #       transfer_status.current_scan_count)
                    print('pre_index    = ', pre_index)
                    print('currentIndex = ', index)
                    if 0 < pre_index:
                        print('currentIndex - pre_index = ',index-pre_index, '\n')
                        data_r = data[pre_index :index]
                    elif 0 > pre_index:
                        print('currentIndex - pre_index = ',total_count+index-pre_index, '\n')
                        data_r = data[pre_index+total_count:total_count]
                        #data_r.append(data[0:index])
                        data_r = data_r + data[0:index]
                
                    print('data_r size = ',len(data_r))
                    print('data size = ',len(data))
                    ch1 = data_r[::4]
                    ch2 = data_r[1::4]
                    ch3 = data_r[2::4]
                    ch4 = data_r[3::4]
                    print('ch1 size = ',len(ch1))
                    print('ch2 size = ',len(ch2))
                    print('ch3 size = ',len(ch3))
                    print('ch4 size = ',len(ch4))
                    print('ch1 max = ',max(ch1))
                    print('threshold = ',np.abs(threshold))
                    inputSignal = np.vstack((np.array(ch1), np.array(ch2), np.array(ch3), np.array(ch4)))
                    #print(rate)
                    pre_index = curr_index
                    
                    

                     
                    
                    
                    if int(np.abs(max(ch1))) >= np.abs(threshold):
                        f = 200 #目標頻率 f = 4000
                        c = 343 #1500
                                                                                                                                    
                        fs = rate
                        M = 1 #目標個數
                        N = 4 #麥克風個數
                        nElements = 4
                        radius = 0.127 #radius = 0.127
                        
                        cutoffLo = 2500 - 500
                        cutoffHi = 2500 + 500
                        order = 6
                        input_filt = np.zeros([4,int(rate/10)],dtype=complex)
                        
                        for i in range(4):
    
                            tem = butter_bandpass_filter(inputSignal[i,:], cutoffLo, cutoffHi, fs, order)
                            input_filt[i,:] = tem

                        inputSignal = input_filt
                        phi = np.arange(0,1,1/nElements)*2*np.pi
                        rho = np.ones((1,nElements))*radius
                        rho = rho[0]
                        #print(rho[0])
                        #print(phi[1])
                        xPos = np.zeros((1,nElements))
                        xPos = xPos[0]
                        yPos = np.zeros((1,nElements))
                        yPos = yPos[0]
                        zPos = np.zeros((1,nElements))
                        zPos = zPos[0]
                        for i in range(nElements):
                            x,y = pol2cart(rho[i],phi[i])
                            xPos[i] = x
                            yPos[i] = y

                        thetaScanAngles = np.linspace(-90,90,361)
                        phiScanAngles = np.linspace(0,180,181)
                        e,u,v,w = steeringVector(xPos,yPos,zPos,f,c,thetaScanAngles,phiScanAngles)

                        R = crossSpectralMatrix(inputSignal)

                        SMUSIC = steeredResponseMUSIC(R,e,w,N,M)

                        spectrumNormalizedMUSIC = np.abs(SMUSIC)/np.max(np.abs(SMUSIC))

                        corr_diff = audio_filt(inputSignal)
                        corr_diff = corr_diff[0]

                        ru,rv = responsePeak(spectrumNormalizedMUSIC,u,v,corr_diff)

                        ang,ele = angcal(ru,rv)
                        hello_str = "ang =  %s" % str(ang)
                        rospy.loginfo(hello_str)
                        pub.publish(hello_str)
                        rate_sleep.sleep()
                    
                    else:
                            
                        
                    
                        hello_str = "not detect" 
                        rospy.loginfo(hello_str)
                        pub.publish(hello_str)
                        rate_sleep.sleep()

                    # Display the data.
                    #print(type(data))
                    # for i in range(channel_count):
                    #     clear_eol()
                    #     print('chan =',
                    #           i + low_channel, ': ',
                    #           '{:.6f}'.format(data[index + i]))

                    #sleep(1)
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


def display_scan_options(bit_mask):
    """Create a displays string for all scan options."""
    options = []
    if bit_mask == ScanOption.DEFAULTIO:
        options.append(ScanOption.DEFAULTIO.name)
    for option in ScanOption:
        if option & bit_mask:
            options.append(option.name)
    return ', '.join(options)


def reset_cursor():
    """Reset the cursor in the terminal window."""
    stdout.write('\033[1;1H')


def clear_eol():
    """Clear all characters to the end of the line."""
    stdout.write('\x1b[2K')


if __name__ == '__main__':
    
    
    try:
        pre_index = 0
        main()
    except rospy.ROSInterruptException:
        pass