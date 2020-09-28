'''
Control the UVBot from a bluetooth client
using pyBluez with python 3
Yao Li: yaoli90@illinois.edu
'''

import bluetooth
import numpy as np
import matplotlib.pyplot as plt
import math

serverMACAddress = 'B8:27:EB:C4:D0:94'
port = 1
s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.connect((serverMACAddress, port))

size = 1000.
MAP_SIZE_PIXELS = 300
MAP_SIZE_METERS = 10


# Receive data from server
# If the data is too large to receive in one time, receive data with length
# 'size' every time.
def receive_data(data, server, size = 1000.):
    for i in range(math.ceil(MAP_SIZE_PIXELS*MAP_SIZE_PIXELS/size)):
        new_data = server.recv(int(size))
        new_data_bytes = bytes(new_data)
        data += new_data_bytes
    return data


if __name__ == "__main__":

    '''
    Commands:

    'm': Display may
    'q': Quit from the client
    'qa' Quit from the client and stop the robot

    '''
    while 1:
        text = input()
        text = bytes(text,'utf-8')

        # unknown commands
        if text not in [b'q', b'qa', b'st', b'sp', b'm',b'fs',b'pp']:
            print('unknown command')
            continue

        if text in [b'st', b'sp']:
            s.send(text)

        if text in [b'fs', b'pp']:
            s.send(text)
        '''
        Quit
        - Command 'q': quit from the client
        - Command 'qa': quit from the client and stop the robot
        '''
        if text in [b'q', b'qa']:
            s.send(text)
            out_file = open("output.txt","wb")
            out_file.write(map)
            out_file.close()
            break

        '''
        Display Map
        - Type command 'm' to display the current map.
        - The map updates once per second. Close the figure to stop the map display
          and go back to command input level.
        '''
        if text == b'm':
            fig = plt.figure()
            FIGURE_OPEN = True
            while FIGURE_OPEN:
                text = b'm'
                s.send(text)

                map = b''
                pos = b''

                pos = receive_data(pos, s) # receive robot position from server
                map = receive_data(map, s) # receive map from server

                # convert bytes to image
                pos_image = np.fromstring(pos, dtype=np.uint8)
                pos_image = pos_image.reshape((MAP_SIZE_PIXELS,MAP_SIZE_PIXELS))

                map_image = np.fromstring(map, dtype=np.uint8)
                map_image = map_image.reshape((MAP_SIZE_PIXELS,MAP_SIZE_PIXELS))

                # construct an image: pos in red, map in blue, green is blank
                # blue: free space in the map
                # white: robot trajectory
                # green: obstacle
                blank = 255*np.zeros((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), dtype=np.uint8)
                image = np.stack((pos_image, blank, map_image), axis=-1)

                plt.clf()
                plt.imshow(image)
                plt.pause(1)
                # check whether the figure is still open
                FIGURE_OPEN = plt.fignum_exists(fig.number)



    s.close()
