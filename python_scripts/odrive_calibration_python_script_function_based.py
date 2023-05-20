""" ODrive Motor Calibration """
""" to install odrive library, run command:

pip install odrive

on terminal/windows cmd """

import odrive
import time
import matplotlib

odrv0 = odrive.find_any()

axis0 = odrv0.axis0 #right leg 0
axis1 = odrv0.axis1 #left leg 1

#state = odrv0.axis0.requested_state

def calibrate_0():
    #AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    axis0.requested_state = 3
    time.sleep(15);
    #axis0.requested_state = 8 #AXIS_STATE_CLOSED_LOOP_CONTROL
    axis0.controller.config.input_mode = 3 #POS_FILTER
    axis0.controller.config.input_filter_bandwidth = 1
    axis0.controller.config.control_mode = 3 #POSITION_CONTROL
    axis0.controller.config.vel_limit = 5
    axis0.controller.config.vel_gain  = 0.165
    pos0 = axis0.encoder.pos_estimate

def calibrate_1():
    axis1.requested_state = 3
    time.sleep(15);
    #axis1.requested_state = 8 
    axis1.controller.config.input_mode = 3
    axis1.controller.config.input_filter_bandwidth = 1 
    axis1.controller.config.control_mode = 3
    axis1.controller.config.vel_limit = 5
    axis1.controller.config.vel_gain  = 0.165
    pos1 = axis1.encoder.pos_estimate
    
#for position control!
#odrv0.axis0.controller.input_pos = 1

#check for errors
#dump_errors(odrv0)

#start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])

calibrate_0()
calibrate_1()

#calibrate_1()

while True:
    #pos0 = 0
    #pos0 = axis0.encoder.pos_estimate
    pos0 = axis0.encoder.pos_estimate
    pos1 = axis1.encoder.pos_estimate
    print(pos0,"\t",pos1)
