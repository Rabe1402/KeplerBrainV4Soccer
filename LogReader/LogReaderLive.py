import serial
import keyboard
import csv
import time
import argparse
from datetime import datetime

bad_event_time = None

def main():
    default_filename = datetime.now().strftime("log_file%d%m%y%H%M.csv")

    parser = argparse.ArgumentParser(description="KeplerBrain BT-Serial Live Logger")
    parser.add_argument("-d", "--device", required=True, help="BT Serial device (z.B. /dev/BTPORT oder COM3)")
    parser.add_argument("-f", "--file", default=default_filename, help="Output CSV Dateiname")
    parser.add_argument("-b", "--baud", type=int, default=9600, help="Baudrate (default: 9600)")
    args = parser.parse_args()

    print(f"Öffne {args.device} mit {args.baud} Baud...")  # ← args.device
    ser = serial.Serial(args.device, args.baud, timeout=1)  # ← args.device, nur einmal

    with open(args.file, 'a') as f:
        while True:
            line = ser.readline().decode().strip()
            print(line)
            f.write(line + '\n')

            if keyboard.is_pressed('space'):
                bad_event_time = line.split('.')[0]
                f.write(f"{bad_event_time} . HUMAN . BAD_EVENT . manual_flag . true\n")
                print(">>> BAD EVENT FLAGGED")

if __name__ == "__main__":  # ← fehlt
    main()