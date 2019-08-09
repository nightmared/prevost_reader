#!/usr/bin/env python3

from typing import List, Type
import serial
import sys
import signal

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


if __name__ == "__main__":
    # Stop capturing data when the user send a SIGINT to the program
    signal.signal(signal.SIGINT, sigint_handler)
    #results = read_from_serial()


    pass
