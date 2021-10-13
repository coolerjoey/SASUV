# Automatic RGB565 Color Tracking Example
#
# This example shows off single color automatic RGB565 color tracking using the OpenMV Cam.

import sensor, image, time, uos, pyb
from pyb import UART, Pin

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()

uart = UART(3, 115200)
red_led = pyb.LED(1) # RED LED on means we are capturing frames.
blue_led = pyb.LED(3) # BLUE LED on means the camera is standing by.
#record_time = 300000 # 300 seconds in milliseconds

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob.pixels() > max_size:
            max_blob = blob
            max_size = blob.pixels()
    return max_blob

threshold = [50, 75, 31, 76, 2, 50]
#start_flag = 88     # Start signal for cam1, X in ASCII
start_flag = 89    # Start signal for cam2, Y in ASCII
record_flag = 82    # Record signal, R in ASCII
red_led.on()
p = Pin('P4', Pin.IN)       # Close TX

while(True):
    img = sensor.snapshot()
    #img.lens_corr(strength = 1.8)      # Lens correction
    blobs = img.find_blobs([threshold], pixels_threshold=5, area_threshold=10, merge=True, margin=10)
    if blobs:
        max_blob = find_max(blobs)
        #img.draw_rectangle(max_blob.rect())            # Drawing for test
        #img.draw_cross(max_blob.cx(), max_blob.cy())       # Drawing for test
        output_str = "%d %d    \r\n" % (max_blob.cx(),max_blob.cy())
        #print(output_str)
    else:
        output_str = "0 0    \r\n"
        #print('0 0    \r\n')
    if uart.any():
        char = uart.readline()
        if int(char[0]) == (start_flag):     # Compare ASCII code
            uart = UART(3, 115200)          # Reinit when receiving start signal
            uart.write(output_str)
            #red_led.off()
            #blue_led.on()
            #blue_led.off()
            #red_led.on()
            p = Pin('P4', Pin.IN)           #Close TX
        elif int(char[0]) == (record_flag):
            record_time1 = int(char[1]) - 48         # Record 1~99 minutes
            record_time2 = int(char[2]) - 48
            img_writer = image.ImageWriter("/stream.bin")
            if record_time2 < 0 or record_time2 > 9:
                record_time = record_time1
            else:
                record_time = record_time1*10 + record_time2
            if record_time < 99 and record_time > 0 :
                record_time = record_time * 60000
                red_led.off()
                blue_led.on()
                start = pyb.millis()
                while pyb.elapsed_millis(start) < record_time:
                    clock.tick()
                    img = sensor.snapshot()
                    img_writer.add_frame(img)
                    print(clock.fps())
                img_writer.close()
                blue_led.off()
                red_led.on()
