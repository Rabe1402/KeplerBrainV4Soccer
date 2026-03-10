import sensor
import image
import pyb
import time
import math

led_green = pyb.LED(2)  # Green LED on the OpenMV Cam
led_red = pyb.LED(1)  # Red LED on the OpenMV Cam
led_blue = pyb.LED(3)  # Blue LED on the OpenMV Cam
led_ir = pyb.LED(4)  # IR LED on the OpenMV Cam

spi_list = [250, 0, 0, 0, 0, 0, 0, 0]  # SPI data list to send to the microcontroller. The first value is the SPI command, the rest are data values.
spi_data = bytearray(spi_list)  # Convert the SPI data list to a bytearray for sending.
threshold_index = 0  # 0 for red
 
def nss_callback(line):
    global spi, spi_data
    try:
        spi.send(spi_data, timeout=1000)
        led_green.on()
        led_red.off()
    except OSError as err:
        led_green.off()
        led_red.on()
        pass
 
pyb.ExtInt(pyb.Pin("P3"), pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, nss_callback)


# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
thresholds = [
  #  (42, 77, 10, 53, 53, 127),
  #  (0, 77, 9, 73, -11, 127),
  #  (0, 77, 9, 73, 13, 127),
  #  (37, 57, 30, 54, 12, 59)
  #  (37, 57, 30, 54, 12, 69)
  #  (11, 41, 4, 50, 1, 28)
    (40, 73, 33, 96, -31, 61)
 
]  # generic_blue_thresholds
 
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  
sensor.set_auto_whitebal(False)  
clock = time.clock()
 
# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.
 
while True:
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs(
    [thresholds[threshold_index]],
    pixels_threshold=20,
    area_threshold=10,
    merge=True,
    )
    biggestblobarea=0
 
    for blob in blobs:
        # These values depend on the blob not being circular - otherwise they will be shaky.
        if blob.elongation() > 0.4:
            img.draw_edges(blob.min_corners(), color=(255, 0, 0))
            img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
            img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
 
        blobarea = blob.area()
        if(blobarea > biggestblobarea):
            biggestblobarea = blobarea
            biggestblob = blob
        # These values are stable all the time.
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        # Note - the blob rotation is unique to 0-180 only.
        img.draw_keypoints(
            [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
        )
        if(biggestblobarea > 0):
            print(str(biggestblob.cx()) + "A" + str(biggestblob.cy()) )
        else:
            print("B")

    #Image Calculations (move as much calculation on to the openmv cam as possible to save time on the microcontroller)
    


    #SPI Data Publishing
    
    #spi_list[1] = <some value calculated from the image>
    #spi_list[2] = <some value calculated from the image>
    #spi_list[3] = <some value calculated from the image>
    #spi_list[4] = <some value calculated from the image>
    #spi_list[5] = <some value calculated from the image>
    #spi_list[6] = <some value calculated from the image>
    #spi_list[7] = <some value calculated from the image>
    spi_data = bytearray(spi_list)  # Update the SPI data bytearray with
    print(spi_data) # Print the SPI data for debugging purposes. #remove this line in production.


    print (clock.fps())  # Print the FPS 