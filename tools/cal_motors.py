#!/usr/bin/env python3

import odrive
from odrive.enums import *

import time
import math
import numpy as np


print('Waiting for drive..')
odrv0 = odrive.find_any()
print('Got drive.. erasing and rebooting..')


odrv0.erase_configuration()
try:
    odrv0.reboot()
except:
    print('Lost connection as expected..')
    

print('Waiting for drive..')
odrv0 = odrive.find_any()
print('Calibrating axis0..')
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

while odrv0.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.encoder.config.cpr = 32768
odrv0.axis0.encoder.config.use_index = False
odrv0.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
odrv0.axis0.controller.config.pos_gain = 50
odrv0.axis0.controller.config.vel_gain = 0.000018
odrv0.axis0.controller.config.vel_limit = 5e6
odrv0.axis0.controller.config.vel_integrator_gain = 0.0
odrv0.axis0.controller.config.pos_integrator_gain = 0.0
odrv0.axis0.config.counts_per_step = 10
odrv0.axis0.config.startup_closed_loop_control = True

print('Calibrating axis1..')
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

while odrv0.axis1.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

odrv0.axis1.encoder.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True
odrv0.axis1.encoder.config.cpr = 32768
odrv0.axis1.encoder.config.use_index = False
odrv0.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
odrv0.axis1.controller.config.pos_gain = 50
odrv0.axis1.controller.config.vel_gain = 0.000018
odrv0.axis1.controller.config.vel_limit = 5e6
odrv0.axis1.controller.config.vel_integrator_gain = 0.0
odrv0.axis1.controller.config.pos_integrator_gain = 0.0
odrv0.axis1.config.counts_per_step = 10
odrv0.axis1.config.startup_closed_loop_control = True
print('Saving config..')
odrv0.save_configuration()

time.sleep(10)
print('Complete')
time.sleep(1)

