import sensor
import image
import pyb
import time
import math

led_green = pyb.LED(2)
led_red = pyb.LED(1)
led_blue = pyb.LED(3)
led_ir = pyb.LED(4)

spi = pyb.SPI(2, pyb.SPI.SLAVE, polarity=0, phase=0)
spi_list = [250, 0, 0, 0, 0, 0, 0, 0]
spi_data = bytearray(spi_list)
threshold_index = 0

# Kamera & Objekt Konstanten
FRAME_W         = 320
FRAME_H         = 240
HFOV_DEG        = 47.5   # echter HFOV mit MT9M114 laut Datenblatt
VFOV_DEG        = 36.6   # echter VFOV
H_FOV_HALF      = HFOV_DEG / 2.0
BALL_DIAMETER_MM = 43    # <-- hier Balldurchmesser anpassen

# Brennweite in Pixel (aus HFOV berechnet)
FOCAL_LEN_PX = (FRAME_W / 2.0) / math.tan(math.radians(H_FOV_HALF))
# ergibt ~364px für diese Linse

# Maximale sinnvolle Distanz für uint8-Skalierung (alles drüber = 255)
MAX_DIST_MM = 3000  # 3 Meter

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
sensor.set_hmirror(True)
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
        norm_x  = (biggestblob.cx() - FRAME_W / 2.0) / (FRAME_W / 2.0)
        angle_h = int(norm_x * H_FOV_HALF)  # -23..+23°

        # Distanz über Pinhole-Formel (blob.w() = Breite des Blobs in Pixeln)
        blob_w_px = max(1, biggestblob.w())  # Division durch 0 vermeiden
        dist_mm   = (BALL_DIAMETER_MM * FOCAL_LEN_PX) / blob_w_px
        dist_mm   = int(max(0, min(MAX_DIST_MM, dist_mm)))

        # Auf uint8 skalieren: 0=weit/kein Ball, 255=sehr nah
        dist_byte = int((1.0 - dist_mm / MAX_DIST_MM) * 255)

        # Rohe Distanz in mm auf zwei Bytes (besser für MCU-Seite!)
        dist_hi = (dist_mm >> 8) & 0xFF
        dist_lo =  dist_mm       & 0xFF

        angle_byte = max(0, min(255, angle_h + 90))

        spi_list[1] = 1           # Ball erkannt
        spi_list[2] = angle_byte  # rel. Winkel (MCU: angle = spi[2] - 90)
        spi_list[3] = dist_hi     # Distanz MSB in mm
        spi_list[4] = dist_lo     # Distanz LSB in mm

        print("Ball: angle={}° dist={}mm".format(angle_h, dist_mm))
    else:
        spi_list[1] = 0   # kein Ball
        spi_list[2] = 90  # Mitte als Default → MCU rechnet 90-90=0, kein Drift
        spi_list[3] = 0
        spi_list[4] = 0

        print("Kein Ball")

    # SPI Data Publishing
    spi_data = bytearray(spi_list)

    print(clock.fps())
