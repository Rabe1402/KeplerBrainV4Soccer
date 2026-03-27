import sensor
import image
import pyb
import time
import math

DEBUG_DRAW = False

led_green = pyb.LED(2)
led_red   = pyb.LED(1)
led_blue  = pyb.LED(3)
led_ir    = pyb.LED(4)

spi = pyb.SPI(2, pyb.SPI.SLAVE, polarity=0, phase=0)
spi_list = [250, 0, 90, 0, 0, 0, 0, 0]
spi_data = bytearray(spi_list)
threshold_index = 0

# Kamera & Objekt Konstanten
FRAME_W          = 320
FRAME_H          = 240
HFOV_DEG         = 71.8
H_FOV_HALF       = HFOV_DEG / 2.0
BALL_DIAMETER_MM = 43

FOCAL_LEN_PX = (FRAME_W / 2.0) / math.tan(math.radians(H_FOV_HALF))

MAX_DIST_MM  = 1500
FRAME_W_HALF = FRAME_W / 2.0

def nss_callback(line):
    global spi, spi_data
    try:
        spi.send(spi_data, timeout=1000)
        led_green.on()
        led_red.off()
    except OSError:
        led_green.off()
        led_red.on()

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
sensor.set_auto_exposure(False)
sensor.set_hmirror(False)
sensor.set_vflip(True)
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs(
        [thresholds[threshold_index]],
        pixels_threshold=20,
        area_threshold=10,
        merge=True,
    )

    if blobs:
        biggestblob = max(blobs, key=lambda b: b.area())

        if DEBUG_DRAW:
            for blob in blobs:
                if blob.elongation() > 0.4:
                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
                img.draw_rectangle(blob.rect())
                img.draw_cross(blob.cx(), blob.cy())
                img.draw_keypoints(
                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
                )

        # Echter Mittelpunkt – korrigiert wenn Ball am Rand abgeschnitten
        blob_x     = biggestblob.x()
        blob_w     = biggestblob.w()
        blob_right = blob_x + blob_w
        est_radius = biggestblob.h() / 2.0

        if blob_x <= 0:
            # Linker Rand abgeschnitten
            true_cx = blob_right - est_radius
        elif blob_right >= FRAME_W:
            # Rechter Rand abgeschnitten
            true_cx = blob_x + est_radius
        else:
            # Ball vollständig sichtbar
            true_cx = biggestblob.cx()

        # atan2 → korrekt auch bei großen Winkeln / nah am Rand
        dx      = true_cx - FRAME_W_HALF
        angle_h = int(math.degrees(math.atan2(dx, FOCAL_LEN_PX)))

        blob_w_px = max(1, blob_w)
        dist_mm   = int(max(0, min(MAX_DIST_MM,
                        (BALL_DIAMETER_MM * FOCAL_LEN_PX) / blob_w_px)))

        spi_list[1] = 1
        spi_list[2] = max(0, min(255, angle_h + 90))
        spi_list[3] = (dist_mm >> 8) & 0xFF
        spi_list[4] =  dist_mm       & 0xFF
        spi_list[5] = 0
        spi_list[6] = 0
        spi_list[7] = 0

        if DEBUG_DRAW:
            print("Ball: angle={}° dist={}mm".format(angle_h, dist_mm))
    else:
        spi_list[1] = 0
        spi_list[2] = 90
        spi_list[3] = 0
        spi_list[4] = 0
        spi_list[5] = 0
        spi_list[6] = 0
        spi_list[7] = 0

        if DEBUG_DRAW:
            print("Kein Ball")

    spi_data = bytearray(spi_list)
    print(clock.fps())