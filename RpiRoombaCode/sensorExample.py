#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# MIT License
# shows how to get sensor data from the create 2

from pycreate2 import Create2
import time


if __name__ == "__main__":
    port = '/dev/ttyUSB0'
    baud = {
        'default': 115200,
        'alt': 19200  # shouldn't need this unless you accidentally set it to this
    }

    bot = Create2(port=port, baud=baud['default'])

    bot.start()

    bot.safe()

    print('Starting ...')

    cnt = 0
    while True:
        # Packet 100 contains all sensor data.
        sensor = bot.get_sensors()

        if cnt%20 == 0:
            print("[L ] [LF] [LC] [CR] [RF] [ R]")

        print(f"{sensor.light_bumper_left:4} {sensor.light_bumper_front_left:4} {sensor.light_bumper_center_left:4} {sensor.light_bumper_center_right:4} {sensor.light_bumper_front_right:4} {sensor.light_bumper_right:4}")
        time.sleep(.01)

        cnt += 1

