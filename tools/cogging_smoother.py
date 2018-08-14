


from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math
import signal
import sys


import numpy as np
import matplotlib.pyplot as plt




class Comp(object):
    def __init__(self, drv):
        self.drv = drv
        self.length = 1204

    def grab(self):
        self.data = np.zeros(self.length)
        for i in range(self.length):
            self.drv.axis0.index = i
            self.data[i] = self.drv.axis0.get_perror()

            
    def process(self):
        wrap = np.argmax(np.fabs(self.data[:-1] - self.data[1:]))
        self.data[wrap + 1:] -= 2 * np.pi
        expected = np.linspace(self.data[0], self.data[-1], self.data.size)
        self.data -= expected

        plt.plot(self.data)
        plt.show()
        
        N = self.data.size
        fft = np.fft.rfft(self.data)
        plt.stem(np.abs(fft[:100]) / N)
        plt.show()
        
        fft_sparse = fft.copy()
        fft_sparse[0] = 0
        fft_sparse[1] = 0
        fft_sparse[2] = 0
        fft_sparse[3] = 0
        fft_sparse[4] = 0
        fft_sparse[5] = 0
        fft_sparse[6] = 0
        plt.stem(np.abs(fft_sparse[:100]) / N)
        plt.show()
        self.interp_data = np.fft.irfft(fft_sparse)
        
        plt.plot(self.interp_data)
        plt.show()
        
    def upload(self):
        print('Uploading data..')
        for i in range(1204):
            self.drv.axis0.index = i
            self.drv.axis0.set_imap(self.interp_data[i])
        print('Data uploaded')
        time.sleep(5)
        self.drv.axis0.mod_depth = 5.0
        print(self.drv.axis0.mod_depth)
        time.sleep(5)
        self.drv.axis0.mod_depth = 10.0
        print(self.drv.axis0.mod_depth)
        time.sleep(5)
        self.drv.axis0.mod_depth = 15.0
        print(self.drv.axis0.mod_depth)
        time.sleep(5)
        self.drv.axis0.mod_depth = 20.0
        print(self.drv.axis0.mod_depth)
        time.sleep(5)
        self.drv.axis0.mod_depth = -5.0
        print(self.drv.axis0.mod_depth)
        time.sleep(5)
        self.drv.axis0.mod_depth = -10.0
        print(self.drv.axis0.mod_depth)
        time.sleep(5)
        self.drv.axis0.mod_depth = -15.0
        print(self.drv.axis0.mod_depth)
        time.sleep(5)
        self.drv.axis0.mod_depth = -20.0
        print(self.drv.axis0.mod_depth)
        print('depths tried')
        
if __name__ == '__main__':

    # Find a connected ODrive (this will block until you connect one)
    print("finding an odrive...")
    odrv0 = odrive.find_any()

    def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        #odrv0.axis0.requested_state = AXIS_STATE_IDLE
        time.sleep(1)
        print('Exiting..')
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    # Find an ODrive that is connected on the serial port /dev/ttyUSB0
    #my_drive = odrive.find_any("serial:/dev/ttyUSB0")

    print("running manual...")
    odrv0.axis0.requested_state = AXIS_STATE_MANUAL_CONTROL
    while odrv0.axis0.current_state != AXIS_STATE_MANUAL_CONTROL:
        time.sleep(0.1)


    # To read a value, simply read the property
    print("Bus voltage is " + str(odrv0.vbus_voltage) + "V")




    d = Comp(odrv0)
    time.sleep(10)
    print('Grabbing data..')
    d.grab()
    print('Plotting..')
    d.process()
    d.upload()
    
    plt.show()

    while True:
        time.sleep(1)
        



