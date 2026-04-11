import serial
import argparse
import csv
from datetime import datetime
import sys 

def main():
    default_filename = datetime.now().strftime("log_file%d%m%y%H%M.csv")

    parser = argparse.ArgumentParser(description="KeplerBrain Serial Logger")
    parser.add_argument("-d", "--device", required=True, help="Serial device (z.B. /dev/ttyUSB0 oder COM3)")
    parser.add_argument("-f", "--file", default=default_filename, help="Output CSV Dateiname(defailt: log-dump.csv)")
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("-t", "--trigger", default="--- Log Dump Start ---", help="Start-Trigger String (default: START_LOG)")
    args = parser.parse_args()

    print(f"Öffne {args.device} mit {args.baud} Baud...")
    print(f"Warte auf Trigger: '{args.trigger}'")

    started = False
    header_written = False
    i = 0

    with serial.Serial(args.device, args.baud, timeout=1) as ser, \
         open(args.file, "w", newline="") as csvfile:

        writer = csv.writer(csvfile)

        try:
            while True:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                if not started:
                    if line == args.trigger:
                        started = True
                        print("Trigger erkannt, starte Logging...")

                    continue

                if started and i == 0:
                    
                    logs = int(line.strip())  # Anzahl der Log-Einträge aus der zweiten zeile lesen
                    print(f"Anzahl der Log-Einträge: {logs}")
                    i = 1
                    continue

                if not header_written:
                    writer.writerow(["index","timestamp_ms","name","component","message","error"])  # Header anpassen
                    header_written = True
                    csvfile.flush()
                if i >= logs:
                    print("\nAlle Log-Einträge wurden erfasst.")
                    sys.exit()

                
                # parts = ["12345", "MainMCU", "StateMachine", "Ball detected"]
                writer.writerow([i] + line.split(" . "))
                  # Log-Eintrag und Zähler in die CSV schreiben
                i += 1
                csvfile.flush()
                print(f"{line}, {i}")

        except KeyboardInterrupt:
            print("\nLogging gestoppt.")

if __name__ == "__main__":
    main()