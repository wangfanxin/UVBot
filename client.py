'''
Control the UVBot from a bluetooth client
using pyBluez with python 3
Yao Li: yaoli90@illinois.edu
'''

import bluetooth
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import math
import time
import struct


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

def send_data(data, server, size = 1000.):
    buffer = bytes(data)
    L = len(buffer)
    for i in range(math.ceil(L/size)):
        server.send(buffer[i*int(size):(i+1)*int(size)])



def onclick(event):
    global ix, iy
    ix, iy = event.xdata, event.ydata
    print (ix, iy)

    global cox,coy,coi
    cox[coi]=ix
    coy[coi]=iy
    coi=coi+1

    if coi == 2:
        fig.canvas.mpl_disconnect(cid)
        plt.close()
    return 1


if __name__ == "__main__":

    '''
    Commands:

    'm': Display may
    'q': Quit from the client
    'qa' Quit from the client and stop the robot

    '''
    map_image = None



    while 1:
        text = input()
        text = bytes(text,'utf-8')

        # unknown commands
        if text not in [b'q', b'qa', b'st', b'sp', b'm', b'lm',b'spt']:
            print('unknown command')
            continue

        if text in [b'st', b'sp']:
            s.send(text)

        '''
        Quit
        - Command 'q': quit from the client
        - Command 'qa': quit from the client and stop the robot
        '''
        if text in [b'q', b'qa']:
            s.send(text)
            break


        if text == b'lm':
            print('Map name?')
            ans = input()
            try:
                map_image = np.asarray(Image.open(ans+'.png'))[:,:,0]
                print('Map - ' + ans + '.png - successfully loaded')
            except:
                print('No map - ' + ans + '.png')
                continue
            s.send(text)
            send_data(map_image.flatten(), s)

        if text == b'spt':
            print('select 2 point for vitual wall using mouse in 5 sec')
            fig = plt.figure()
            FIGURE_OPEN = True

            text = b'spt'
            s.send(text)

            map = b''
            pos = b''

            #pos = receive_data(pos, s) # receive robot position from server
            map = receive_data(map, s) # receive map from server

            # convert bytes to image
            pos_image = 255*np.zeros((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), dtype=np.uint8)

            map_image = np.fromstring(map, dtype=np.uint8)
            map_image = map_image.reshape((MAP_SIZE_PIXELS,MAP_SIZE_PIXELS))

            # construct an image: pos in red, map in blue, green is blank
            # blue: free space in the map
            # white: robot trajectory
            # green: obstacle
            blank = 255*np.zeros((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), dtype=np.uint8)
            image = np.stack((pos_image, blank, map_image), axis=-1)

            time.sleep(1)

            cox=np.array([0.0,0.0])
            coy=np.array([0.0,0.0])
            coi=0

            plt.clf()
            plt.imshow(image)
            cid = fig.canvas.mpl_connect('button_press_event', onclick)

            plt.pause(8)

            time.sleep(8)

            cox_=cox*1000*MAP_SIZE_METERS/MAP_SIZE_PIXELS
            cox_=cox_-5000

            coy_=coy*1000*MAP_SIZE_METERS/MAP_SIZE_PIXELS
            coy_=coy_-5000

            print(struct.pack('f',cox_[0]))
            print(type(struct.pack('f',cox_[0])))
            print(type(text))
            s.send(struct.pack('f',cox_[0]))
            time.sleep(1)
            s.send(struct.pack('f',coy_[0]))
            time.sleep(1)
            s.send(struct.pack('f',cox_[1]))
            time.sleep(1)
            s.send(struct.pack('f',coy_[1]))
            time.sleep(1)

            print(cox_)
            print(coy_)

            time.sleep(5)

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

                plt.plot(cox,coy,'-')

                plt.pause(1)
                # check whether the figure is still open
                FIGURE_OPEN = plt.fignum_exists(fig.number)

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
    # save the map
    if map_image is not None:
        print('Save map (<name>/n)?')
        ans = input()
        if ans != 'n':
            plt.imsave(ans+'.png',np.stack([map_image, map_image, map_image], axis=-1))
