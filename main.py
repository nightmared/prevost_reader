#!/usr/bin/env python3

from typing import List, Type
from datetime import datetime
import serial
import sys
import signal
import csv

import config
import lib

continue_capturing_data = False

def read_from_serial() -> List[Type[lib.Measurement]]:
    global continue_capturing_data
    continue_capturing_data = True

    results = []

    try:
        ser = serial.Serial(config.SERIAL_PORT)
    except:
        raise AssertionError(f"""Failed to open the serial device {config.SERIAL_PORT}, exiting...""")
    ser.timeout = 0.2
    print(f"""Reading values from the serial port {ser.name}. Press ^C to interrupt the capture""")
    while continue_capturing_data:
        line = ser.readline()[:-1]
        pos = line.split(',')
        if len(pos) == 3:
            try:
                cam = int(pos[0])+1
                x = int(pos[1])
                y = int(pos[2])
                results.append(Measurement(config.CAMERAS[cam], Vec2D(x, y)))
            except ValueError:
                pass

    ser.close()
    return results

def sigint_handler(*args):
    global continue_capturing_data
    continue_capturing_data = False

# Return the file name
def measure() -> str:
    # Stop capturing data when the user send a SIGINT to the program
    signal.signal(signal.SIGINT, sigint_handler)

    results = read_from_serial()
    with open(f"""measurements/{datetime.utcnow().isoformat()}.csv""", 'w') as f:
        fieldnames = ['camera', 'x', 'y']
        writer = csv.DictWriter(f, fieldsnames=fieldnames)
        writer.writeheader()
        for m in results:
            writer.writerow({'camera': m.camera, 'x': m.position.x, 'y': m.position.y})

    # Reset the signal handler to the default one
    signal.signal(signal.SIGINT, signal.SIG_DFL)

def open_measurement(file_name: str) -> List[Type[lib.Measurement]]:
    results = []
    with open(file_name, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            results.append(Measurement(config.CAMERAS[row['camera']], Vec3D(row['x'], row['y'], 0)))
    return results

if __name__ == "__main__":
    #measurements = open_measurement(measure())
    pass
