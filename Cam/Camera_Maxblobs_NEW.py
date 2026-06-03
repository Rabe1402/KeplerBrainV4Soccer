import pyb
import sensor
import math
import time 
import machine
import utime
import image

DEBUG_DRAW = True

spi = pyb.SPI(2, pyb.SPI.SLAVE, polarity=0, phase=0)
led_red = pyb.LED(1)
led_green = pyb.LED(2)
spi_list = [250, 1, 2, 3, 4, 5, 6, 7]
spi_data = bytearray(spi_list)
largest_blob = 0
img = 0

def nss_callback(line):
    global spi, spi_data
    try:
        spi.send(spi_data, timeout=1000)
        led_green.on()
        led_red.off()
    except:
        led_green.off()
        led_red.on()
        pass

pyb.ExtInt(pyb.Pin("P3"), pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, nss_callback)


# setup camera
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(10)
#blue
blueThreshold = (31, 51, -8, 12, -45, -13)#(39, 65, -21, 6, -78, -18)
#yellow
yellowThreshold = (79, 100, -33, 9, 43, 74)#(72, 95, -18, 14, 27, 51)
#orange
orangeThreshold = (39, 100, 35, 54, 32, 68)#(43, 100, 4, 74, 34, 75)


wdt = machine.WDT(timeout=5000)

def findBlob(threshold):
    largest_blob = 0

    blobs = img.find_blobs([threshold])
    for b in blobs:
        if DEBUG_DRAW:
            img.draw_rectangle(b[0:4])
        #print("x:%d y:%d w:%d h:%d" % (b[0],b[1],b[2],b[3]))
        largest_blob = max(blobs, key =lambda b: b.pixels())

    if largest_blob == 0:
        return 0

    if largest_blob:
        if DEBUG_DRAW:
            img.draw_rectangle(largest_blob.rect())

    lb = [0, 0, largest_blob[2], largest_blob[3]]
    lb[0] = largest_blob[0] + int(largest_blob[2] / 2)
    lb[1] = largest_blob[1] + int(largest_blob[3] / 2)
    return lb

def transformBlob(blob, n):

    if blob == 0:
        spi_list[n] = 0
        spi_list[n + 1] = 0
    else:
        #angle
        spi_list[n] = blob[0]
        #distance
        spi_list[n + 1] = blob[1]


while(True):
    wdt.feed()
    clock.tick()
    img = sensor.snapshot()
    roisOrange = findBlob(orangeThreshold)
    roisBlue = findBlob(blueThreshold)
    roisYellow = findBlob(yellowThreshold)


    transformBlob(roisOrange, 5)
    transformBlob(roisBlue, 1)
    transformBlob(roisYellow, 3)

    spi_data = bytearray(spi_list)
    
    
    print(clock.fps())
