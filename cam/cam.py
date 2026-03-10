import sensor
import image
import pyb
import time
import math

led_green = pyb.LED(2)
led_red = pyb.LED(1)
led_blue = pyb.LED(3)
led_ir = pyb.LED(4)

spi_list = [250, 0, 0, 0, 0, 0, 0, 0]
spi_data = bytearray(spi_list)
threshold_index = 0

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

thresholds = [
    (40, 73, 33, 96, -31, 61)
]

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()

FRAME_W    = 320
FRAME_H    = 240
H_FOV_HALF = 35.0  # halbes horizontales FOV (~70° gesamt)

while True:
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs(
        [thresholds[threshold_index]],
        pixels_threshold=20,
        area_threshold=10,
        merge=True,
    )

    biggestblobarea = 0
    biggestblob = None

    for blob in blobs:
        if blob.elongation() > 0.4:
            img.draw_edges(blob.min_corners(), color=(255, 0, 0))
            img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
            img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))

        blobarea = blob.area()
        if blobarea > biggestblobarea:
            biggestblobarea = blobarea
            biggestblob = blob

        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        img.draw_keypoints(
            [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
        )

    # Image Calculations
    if biggestblobarea > 0 and biggestblob is not None:
        # Relativer Winkel: negativ = Ball links, positiv = Ball rechts
        norm_x  = (biggestblob.cx() - FRAME_W / 2.0) / (FRAME_W / 2.0)
        angle_h = int(norm_x * H_FOV_HALF)  # ca. -35 bis +35°

        # Distanzschätzung über Blob-Höhe (größerer Blob = näher = höherer Wert)
        dist_byte = max(0, min(255, int((biggestblob.h() / FRAME_H) * 255)))

        # Area auf zwei Bytes
        area_clamped = min(biggestblobarea, 65535)
        area_hi = (area_clamped >> 8) & 0xFF
        area_lo =  area_clamped       & 0xFF

        # Winkel mit Offset +90 als uint8 senden (90 = geradeaus)
        angle_byte = max(0, min(255, angle_h + 90))

        spi_list[1] = 1           # Ball erkannt
        spi_list[2] = angle_byte  # rel. Winkel (MCU: angle_h = spi[2] - 90)
        spi_list[3] = dist_byte   # Distanzschätzung
        spi_list[4] = area_hi     # Area MSB
        spi_list[5] = area_lo     # Area LSB
        spi_list[6] = max(0, min(255, int(biggestblob.elongation() * 100)))
        spi_list[7] = 0           # reserviert

        print("Ball: angle={} dist={}".format(angle_h, dist_byte))
    else:
        spi_list[1] = 0   # kein Ball
        spi_list[2] = 90  # Mitte als Default → MCU rechnet 90-90=0, kein Drift
        spi_list[3] = 0
        spi_list[4] = 0
        spi_list[5] = 0
        spi_list[6] = 0
        spi_list[7] = 0

        print("Kein Ball")

    # SPI Data Publishing
    spi_data = bytearray(spi_list)

    print(clock.fps())
    