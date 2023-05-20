import odrive
import time

odrv0 = odrive.find_any()

axis0 = odrv0.axis0

#state = odrv0.axis0.requested_state

axis0.controller.config.input_filter_bandwidth = 5

pos = axis0.encoder.pos_estimate
time.sleep(1)

max_pos = pos + 0.25
min_pos = pos - 0.25
#min_pos = pos - 3.375

print(pos)
n = 0.5

count = 0

while True:
    time.sleep(0.25)
    print(pos)
    pos = pos + n
    axis0.controller.input_pos = pos

    if axis0.encoder.pos_estimate > max_pos - 0.05: 
        n = -1
    elif axis0.encoder.pos_estimate < min_pos + 0.05:
        n = 1



