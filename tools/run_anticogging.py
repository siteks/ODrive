
import odrive
from odrive.enums import *
import time
import math
import numpy as np
import matplotlib.pyplot as plt

# oscillations at 22808

print('Waiting for drive..')
odrv0 = odrive.find_any()
print('Got drive..')
print('Calibrating axis 0..')
odrv0.axis0.controller.config.vel_gain = 0.00015
odrv0.axis0.controller.config.pos_integrator_gain = 0
odrv0.axis0.controller.config.vel_integrator_gain = 0.002
odrv0.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.start_anticogging_calibration()

while not odrv0.axis0.controller.config.use_anticogging:
    print(odrv0.axis0)
    time.sleep(0.2)

odrv0.axis0.requested_state = AXIS_STATE_IDLE

print('Anticogging cal complete, fetching map..')
cm0_raw = np.zeros((4096))
for i in range(4096):
    cm0_raw[i] = odrv0.axis0.controller.read_cogging_map(i)

np.save('cm0_raw.npy', cm0_raw)
fft = np.fft.rfft(cm0_raw)
fft[0] = 0
fft[1] = 0
fft[2] = 0
fft[3] = 0
fft[4] = 0


print('Sending first 128 components..')
for i in range(128):
    re = np.real(fft[i])
    im = np.imag(fft[i])
    odrv0.axis0.controller.write_fft_map(i * 2, re)
    odrv0.axis0.controller.write_fft_map(i * 2 + 1, im)

print('Recalculate cogging map..')
odrv0.axis0.controller.calc_cogging_map()

while not odrv0.axis0.controller.valid_cogging_map:
    print('Waiting..')
    time.sleep(1)

print('Fetching new cogging map..')
cm0_filt = np.zeros((4096))
for i in range(4096):
    cm0_filt[i] = odrv0.axis0.controller.read_cogging_map(i)


np.save('cm0_filt.npy', cm0_raw)


print('Calibrating axis 1..')
odrv0.axis1.controller.config.vel_gain = 0.00015
odrv0.axis1.controller.config.pos_integrator_gain = 0
odrv0.axis1.controller.config.vel_integrator_gain = 0.002
odrv0.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.controller.start_anticogging_calibration()

while not odrv0.axis1.controller.config.use_anticogging:
    print(odrv0.axis1)
    time.sleep(0.2)

odrv0.axis1.requested_state = AXIS_STATE_IDLE

print('Anticogging cal complete, fetching map..')
cm1_raw = np.zeros((4096))
for i in range(4096):
    cm1_raw[i] = odrv0.axis1.controller.read_cogging_map(i)

np.save('cm1_raw.npy', cm1_raw)
fft = np.fft.rfft(cm1_raw)
fft[0] = 0
fft[1] = 0
fft[2] = 0
fft[3] = 0
fft[4] = 0


print('Sending first 128 components..')
for i in range(128):
    re = np.real(fft[i])
    im = np.imag(fft[i])
    odrv0.axis1.controller.write_fft_map(i * 2, re)
    odrv0.axis1.controller.write_fft_map(i * 2 + 1, im)

print('Recalculate cogging map..')
odrv0.axis1.controller.calc_cogging_map()

while not odrv0.axis1.controller.valid_cogging_map:
    print('Waiting..')
    time.sleep(1)

print('Fetching new cogging map..')
cm1_filt = np.zeros((4096))
for i in range(4096):
    cm1_filt[i] = odrv0.axis1.controller.read_cogging_map(i)


np.save('cm1_filt.npy', cm1_raw)



