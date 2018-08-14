

import numpy as np
import matplotlib.pyplot as plt


# prepare with:
# get address of array:
# p (char*)&pos_map + 0x8000
# dump memory to file:
# dump binary memory /Volumes/Data/simonj/pos_map.bin 0x200032ac 0x2000b2ac



# phase at start
scan_omega = 4 * np.pi
scan_distance = 16 * np.pi
current_meas_hz = 8241
num_steps = (scan_distance / scan_omega * current_meas_hz)
pole_pairs = 7

def ph(i):
    return (scan_distance * i / num_steps - scan_distance / 2.0) / pole_pairs


angles = np.linspace(ph(0), ph(32768), 16384)
angles -= angles[0]

steps_per_rad = - 32768 / (2 * np.pi)



data = np.fromfile(open('phase_error.bin','rb'),np.dtype(np.float32)).astype(np.float64)


plt.ion()

wrap = np.argmax(np.fabs(data[:-1] - data[1:]))
data[wrap + 1:] -= 2 * np.pi


#
#data =[ np.fromfile(open('pos_map_5A_cal.bin','rb'),np.dtype(np.int16)).astype(np.float64),
#        np.fromfile(open('pos_map_10A_cal.bin','rb'),np.dtype(np.int16)).astype(np.float64),
#        np.fromfile(open('pos_map_20A_cal.bin','rb'),np.dtype(np.int16)).astype(np.float64)]
#
#for d in data:
#    d -= d[0]
#    d /= steps_per_rad
#
## find point closest to one revolution
#rp = np.argmin(np.fabs(angles-2*np.pi))
#
## make new array and put data in it
#d2 = np.zeros((rp, 4))
#d2[:,0] = angles[:rp]
#d2[:,1] = data[0][:rp] - d2[:,0]
#d2[:,2] = data[1][:rp] - d2[:,0]
#d2[:,3] = data[2][:rp] - d2[:,0]
#
#
#def clean_data(x):
#    N=x.size
#    fft = np.fft.rfft(x)
#
#    fft_sparse = fft.copy()
#    fft_sparse[0] = 0
#    fft_sparse[1] = 0
#    fft_sparse[2] = 0
#    fft_sparse[3] = 0
#    fft_sparse[4] = 0
#    fft_sparse[6] = 0
#
#    interp_data = np.fft.irfft(fft_sparse)
#    return interp_data
#
#
#
##
##freq = np.fft.rfftfreq(N, d=1/N)
#
#i1 = clean_data(d2[:,1])
#i2 = clean_data(d2[:,2])
#i3 = clean_data(d2[:,3])
#
#
#plt.ion()
#plt.plot(d2[:i1.size,0], i1, label='5A')
#plt.plot(d2[:i1.size,0], i2, label='10A')
#plt.plot(d2[:i1.size,0], i3, label='20A')
#plt.legend()
#
#plt.show()
#
#
