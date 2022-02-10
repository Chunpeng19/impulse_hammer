#!/usr/bin/env python

import rospy
import numpy as np
import time
import uldaq
from uldaq import (get_daq_device_inventory, DaqDevice, InterfaceType,
                   AiInputMode, AInFlag)
from impulse_hammer.msg import force

class hammer_node():
    daq_device = None

    range_index = 0
    channel = 0
    vol_vs_force_ratio = 0.0112

    rospy.init_node('impulse_hammer')
    sample_rate = 500
    rate = rospy.Rate(sample_rate)

    pub = rospy.Publisher('impulse_force',force,queue_size=1)

    def __init__(self):
        rospy.on_shutdown(self.safe_shutdown)

        try:
            descriptors = uldaq.get_daq_device_inventory(uldaq.InterfaceType.USB, number_of_devices=1)
            self.daq_device = uldaq.DaqDevice(descriptors[0])
            self.ai_device = self.daq_device.get_ai_device()
            self.daq_device.connect(connection_code=0)
            self.ai_info = self.ai_device.get_info()
            self.input_mode = AiInputMode.DIFFERENTIAL
            self.ranges = self.ai_info.get_ranges(self.input_mode)
            for index in range(0, len(self.ranges)):
                if self.ranges[index] == 5: # 10V
                    self.range_index = index
                    print('+-10V output range selected.')
            print('Sensor ininitalized.')

        except RuntimeError as error:
            print('\n',error)
            self.safe_shutdown

    def safe_shutdown(self):
        if self.daq_device:
            if self.daq_device.is_connected():
                self.daq_device.disconnect()
            self.daq_device.release()

    def loop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            voltage = self.ai_device.a_in(self.channel,self.input_mode,self.ranges[self.range_index],AInFlag.DEFAULT)
            force = voltage / self.vol_vs_force_ratio
            self.pub.publish(force)

if __name__ == "__main__":
    node = hammer_node()
    node.loop()
