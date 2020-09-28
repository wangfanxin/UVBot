import threading
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from Lidar import Lidar, start_lidar
import time
import bluetooth
import math
import numpy as np
import pyudev
from pycreate2 import Create2
import sys, select
import copy
import struct
# Basic params
_DEFAULT_MAP_QUALITY         = 2 # out of 255
_DEFAULT_HOLE_WIDTH_MM       = 600

# Random mutation hill-climbing (RMHC) params
_DEFAULT_SIGMA_XY_MM         = 80
_DEFAULT_SIGMA_THETA_DEGREES = 8
_DEFAULT_MAX_SEARCH_ITER     = 1000

_LIGHT_MAX = 800 #4096.0/4*3


'''
Thread functions:
'''
def run_slam(LIDAR_DEVICE, MAP_SIZE_PIXELS, MAP_SIZE_METERS, mapbytes, posbytes, odometry, command, state, STATES):
    MIN_SAMPLES   = 200

    # Connect to Lidar unit
    lidar = start_lidar(LIDAR_DEVICE)
    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS,map_quality=_DEFAULT_MAP_QUALITY, hole_width_mm=_DEFAULT_HOLE_WIDTH_MM,
                random_seed=None, sigma_xy_mm=_DEFAULT_SIGMA_XY_MM, sigma_theta_degrees=_DEFAULT_SIGMA_THETA_DEGREES,
                max_search_iter=_DEFAULT_MAX_SEARCH_ITER)
    # Initialize empty map
    mapbytes_ = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    # Send command to start scan
    lidar.start_scan()
    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = None
    previous_angles    = None

    pose_change = (0,0,0)
    pose_change_find = (0,0,0)
    iter_times=1
    while command[0] != b'qa':

        # read a scan
        items = lidar.scan()

        # Extract distances and angles from triples
        distances = [item[2] for item in items]
        angles    = [item[1] for item in items]



        if state[0] == STATES['stop']:
            if command[0] == b'lm':
                mapbytes_find=mapbytes

                pose_change_find=find_slam_pos(lidar, MAP_SIZE_PIXELS, MAP_SIZE_METERS, mapbytes_find, pose_change_find,iter_times/2.0,slam)
                iter_times=iter_times+1.0
                print(pose_change_find)
                pose_change=pose_change_find
                pose_change_find=(0,0,0)

                '''
                items = lidar.scan()

        # Extract distances and angles from triples
                distances = [item[2] for item in items]
                angles    = [item[1] for item in items]

                slam.update(distances, pose_change, scan_angles_degrees=angles)
                '''

                if(iter_times>10):
                    command[0] = b'w'
                    print(slam)

                    slam.setmap(mapbytes_find)
                    pose_change=pose_change_find
                    pose_change_find=(0,0,0)
                    print('ennnnnnnnnnnnnnnnnnd')
                    print(pose_change)
                    iter_times=1


        else:

            # Update SLAM with current Lidar scan and scan angles if adequate
            if len(distances) > MIN_SAMPLES:
                slam.update(distances, pose_change, scan_angles_degrees=angles)
                previous_distances = distances.copy()
                previous_angles    = angles.copy()

            # If not adequate, use previous
            elif previous_distances is not None:
                slam.update(previous_distances, pose_change, scan_angles_degrees=previous_angles)

            #pass the value into point_cal function
            global point_a,point_b
            global current_theta_o
            # Get current robot position
            x, y, theta = slam.getpos()
            current_theta_o = theta
            point_a=x-5000
            point_b=y-5000


            # Calculate robot position in the map
            xbytes = int(x/1000./MAP_SIZE_METERS*MAP_SIZE_PIXELS)
            ybytes = int(y/1000./MAP_SIZE_METERS*MAP_SIZE_PIXELS)
            # Update robot position
            posbytes[xbytes + ybytes*MAP_SIZE_PIXELS] = 255
            #print('LIDAR::', command[0], x,y,theta)
            pose_change = (odometry[0], odometry[1]/np.pi*180, odometry[2])
            # Get current map bytes as grayscale
            slam.getmap(mapbytes_)
            # Update map
            mapbytes[:] = mapbytes_




    # Shut down the lidar connection
    lidar.stop()
    lidar.disconnect()

    print('LIDAR::thread ends')

def find_slam_pos(lidar, MAP_SIZE_PIXELS, MAP_SIZE_METERS, mapbytes_find, pose_change_find,iter_times,find_slam):
    MIN_SAMPLES   = 200
    #slam.sigma_xy_mm=300/iter_times
    #slam.sigma_theta_degrees=30/iter_times
    #find_slam=slam
    find_slam.sigma_xy_mm         =  _DEFAULT_SIGMA_XY_MM#300/iter_times
    find_slam.sigma_theta_degrees =   _DEFAULT_SIGMA_THETA_DEGREES#30/iter_times
    #find_slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS,map_quality=_DEFAULT_MAP_QUALITY, hole_width_mm=_DEFAULT_HOLE_WIDTH_MM,
    #            random_seed=None, sigma_xy_mm=300/iter_times, sigma_theta_degrees=30/iter_times,
    #            max_search_iter=_DEFAULT_MAX_SEARCH_ITER)
    # Initialize empty map
    find_slam.setmap(mapbytes_find)
    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = None
    previous_angles    = None
    #pose_change = (odometry[0], odometry[1]/np.pi*180, odometry[2])
    #pose_change_find = (0,0,0)
    find_iter=0
    while find_iter<1:
        # read a scan
        items = lidar.scan()

        # Extract distances and angles from triples
        distances = [item[2] for item in items]
        angles    = [item[1] for item in items]

        # Update SLAM with current Lidar scan and scan angles if adequate
        if len(distances) > MIN_SAMPLES:
            find_slam.update(distances, pose_change_find, scan_angles_degrees=angles)
            previous_distances = distances.copy()
            previous_angles    = angles.copy()

        # If not adequate, use previous
        elif previous_distances is not None:
            find_slam.update(previous_distances, pose_change_find, scan_angles_degrees=previous_angles)

        # Get current robot position
        x, y, theta = find_slam.getpos()
        x=x-5000
        y=y-5000
        pose_change_find=(np.sqrt(x*x+y*y),theta,0)

        find_iter=find_iter+1
    # Shut down the lidar connection


    print('LIDAR::thread ends')
    return pose_change_find


def run_bluetooth(hostMACAddress, mapbytes, posbytes, command, state, STATES):
    port = 1
    backlog = 1

    global x1,y1,x2,y2
    x1=(0,0)
    x2=(0,0)
    y1=(0,0)
    y2=(0,0)
    while command[0] != b'qa':

        # loop and wait for client to connect
        print('BLUETOOTH::waiting for bluetooth connection ...')
        s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        s.bind((hostMACAddress, port))
        s.listen(backlog)
        try:
            client, clientInfo = s.accept()
            print('BLUETOOTH::connected')

            while command[0] != b'qa':
                data = client.recv(10)
                print('BLUETOOTH::info:', data)
                print('command',command[0])
                # send map
                if data == b'm':
                    send_data(posbytes, client)
                    send_data(mapbytes, client)

                # quit all
                elif data == b'qa':
                    state[0] = STATES['stop']
                    command[0] = b'qa'

                elif data == b'st':
                    state[0] = STATES['find_wall']

                elif data == b'sp':
                    state[0] = STATES['stop']

                elif data == b'lm':
                    state[0] = STATES['stop']
                    command[0] = b'lm'
                    time.sleep(1)
                    receive_data(mapbytes, client)
                elif data == b'spt':
                    state[0] = STATES['stop']


                    send_data(mapbytes, client)

                    command[0] = b'spt'

                    time.sleep(1)

                    x1 = client.recv(10)
                    print(x1)
                    y1 = client.recv(10)
                    print(y1)
                    x2 = client.recv(10)
                    print(x2)
                    y2 = client.recv(10)
                    print(y2)

                    x1=struct.unpack('f',x1)
                    y1=struct.unpack('f',y1)
                    x2=struct.unpack('f',x2)
                    y2=struct.unpack('f',y2)

                    print(x1,y1,x2,y2)
                    state[0]=STATES['find_wall']

                elif data:
                    client.send(data)
        except:	
            print("BLUETOOTH::closing socket")
            client.close()
            s.close()

    print('BLUETOOTH::thread ends')


def run_robot(ROBOT_DEVICE, odometry, command, state, STATES):
    # create Create2 object
    bot = Create2(port=ROBOT_DEVICE, baud=115200)
    # start robot
    bot.start()
    # robot drive in safe mode (cliff and wheel drop detection on)
    bot.safe()
    play_start_music(bot)
    print('ROBOT::start')

    while command[0] != b'qa':

        # if start robot, go find wall
        if state[0] is STATES['find_wall']: # if in find wall mode, go find the wall
            find_wall(bot, odometry, command, state, STATES)
        # if wall reached, go follow wall
        elif state[0] is STATES['follow_wall']:
            follow_wall(bot, odometry, command, state, STATES) # o.w. follow the wall


    print('ROBOT::thread ends')

'''
Bluetooth functions:
'''
# Send data to client
# If the data is too large to send in one time, send data with length 'size' every time
def send_data(data, client, size=1000.):
    buffer = bytes(data)
    L = len(buffer)
    for i in range(math.ceil(L/size)):
        client.send(buffer[i*int(size):(i+1)*int(size)])

def receive_data(data, client, size=1000.):
    buffer = bytes(data)
    L = len(buffer)
    for i in range(math.ceil(L/size)):
        new_data = client.recv(int(size))
        data[i*int(size):(i+1)*int(size)] = new_data

'''
Initialize functions:
'''
# Find irobot usb device and lidar usb device
def find_device():
    context = pyudev.Context()
    for device in context.list_devices(subsystem='tty'):
        if device.get('ID_MODEL_ID') == 'ea60':
            LIDAR_DEVICE = device.get('DEVNAME')
        elif device.get('ID_MODEL_ID') == '6015':
            ROBOT_DEVICE = device.get('DEVNAME')
    print('ROBOT_DEVICE:', ROBOT_DEVICE)
    print('LIDAR_DEVICE:', LIDAR_DEVICE)
    return ROBOT_DEVICE, LIDAR_DEVICE

'''
Robot functions:
'''
# robot play start music
def play_start_music(bot):
    song = [76, 12, 76, 12, 20, 12, 76, 12, 20, 12, 72, 12, 76, 12, 20, 12, 79, 12, 20, 36, 67, 12, 20, 36]
    song_num = 3  # song number can be 0-3
    bot.createSong(song_num, song)
    time.sleep(0.1)
    how_long = bot.playSong(song_num) # how_long is the time in secods for it to finish
    time.sleep(how_long)


def find_wall(bot, odometry, command, state, STATES):

    path = [[  0,  0, 0.1, 'strt']] # init path

    __, __, encoder = bot.get_sensors_uvbot() # read sensor
    encoder_last = np.array([encoder['L'], encoder['R']]) # remember initial encoder reading

    # find wall if robot is not issued to remote control or stop
    # terminates when a wall is reached
    while state[0] not in [STATES['stop'], STATES['remote'], STATES['follow_wall']]:

        # robot move on 'path'
        for lft, rht, dt, s in path:
            bot.digit_led_ascii(s)
            bot.drive_direct(lft, rht)
            time.sleep(dt)


        light, bump, encoder = bot.get_sensors_uvbot() # read sensor
        # print('ROBOT::', light, bump, state[0], path[0][:2])

        # calculate robot position and orientation
        encoder_new = np.array([encoder['L'], encoder['R']]) - encoder_last # encoder incremental
        encoder_new += (encoder_new > 1e4) * (-65536) + (encoder_new < -1e4) * 65536 # remove overflow

        encoder_last = np.array([encoder['L'], encoder['R']]) # update restored encoder reading
        encoder_new_mm = encoder_new * np.pi * 72.0 / 508.8 # encoder incremental in mm
        ang_new = (encoder_new_mm[1] - encoder_new_mm[0]) / 235.0  # orientation incremental in rad

        odometry[0] = np.mean(encoder_new_mm)
        odometry[1] = ang_new
        odometry[2] = path[0][2]


        # find a wall
        if state[0] == STATES['find_wall']:
            if not bump['any']: # if not reach a wall, go forward
                path = [[ 20, 20, 0.1, 'forw']]
            else: # if reached a wall, backward a bit
                state[0] = STATES['reach_wall']
                path = [[-20,-20, 0.7, 'back']]
                print('ROBOT::reach wall')
            continue

        # rotate to a good direction for wall following
        if state[0] == STATES['reach_wall']:
            if light['R'] < 400: # rotate until right sensor find a wall
                path = [[40, -40, 0.1, 'left']]
            else: # pasue 1 sec. before follow the wall
                path = [[  0,  0,   1, 'paus']]
                state[0] = STATES['follow_wall']
                print('ROBOT::follow wall')
            continue


    # quit from find wall
    path = [[ 0,0, 0.1, 'stop']] # stop robot, prevent the robot keep moving under previous 'path'
    for lft, rht, dt, s in path:
            bot.digit_led_ascii(s)
            bot.drive_direct(lft, rht)
            time.sleep(dt)

def follow_wall(bot, odometry, command, state, STATES):
    global point_a,point_b,x1,y1,x2,y2

    path = [[  0,  0, 0.1, 'strt']] # init path

    __, __, encoder = bot.get_sensors_uvbot() # read sensor
    encoder_last = np.array([encoder['L'], encoder['R']]) # remember initial encoder reading
    forward_flag=0
    ppd_old=1000
    ppd=1000
    round_flag = 1
    vitual_wall_flag=1
    global current_theta_o
    current_theta_o=0
    # follow wall if robot is not issued to remote control or stop or find a wall
    while state[0] not in [STATES['stop'], STATES['find_wall'], STATES['remote']]:

        # robot move on 'path'
        for lft, rht, dt, s in path:
            bot.digit_led_ascii(s)
            bot.drive_direct(lft, rht)
            time.sleep(dt)

        light, bump, encoder = bot.get_sensors_uvbot() # read from sensor
        # print(light, path[0][:2], encoder_last)

        # calculate robot position and orientation
        encoder_new = np.array([encoder['L'], encoder['R']]) - encoder_last # encoder incremental
        encoder_new += (encoder_new > 1e4) * (-65536) + (encoder_new < -1e4) * 65536 # remove overflow

        encoder_last = np.array([encoder['L'], encoder['R']]) # update restored encoder reading
        encoder_new_mm = encoder_new * np.pi * 72.0 / 508.8 # encoder incremental in mm
        ang_new = (encoder_new_mm[1] - encoder_new_mm[0]) / 235.0  # orientation incremental in rad

        odometry[0] = np.mean(encoder_new_mm)
        odometry[1] = ang_new
        odometry[2] = path[0][2]

        '''
        pos[2] += ang_new # update current orientation

        if ang_new:
            pos[:2] = pos[:2] + np.mean(encoder_new_mm) * (np.sin(ang_new)/ang_new * np.array([np.cos(pos[2]), np.sin(pos[2])]) + (1-np.cos(ang_new))/ang_new * np.array([np.sin(pos[2]), -np.cos(pos[2])]))
        else:
            pos[:2] = pos[:2] + np.mean(encoder_new_mm)*np.array([np.cos(pos[2]), np.sin(pos[2])]) # update current position
        '''
        # virtual wall handling

        a=point_a
        b=point_b
        #current_theta
        current_theta=current_theta_o+1.0
        #print(current_theta_o)
        current_theta=current_theta % 360
        if current_theta>180:
            current_theta=current_theta-360
        elif current_theta<-180:
            current_theta=current_theta+360
        #print(current_theta)

        line_theta=np.arctan2(float(y2[0])-float(y1[0]),float(x2[0])-float(x1[0]))/np.pi*180
        flag=0
        if command[0] == b'spt':
            c,d,ppd,flag=point_cal(x1,y1,x2,y2,a,b)
            print(c,d,ppd,flag)
        if ppd_old > ppd:
            forward_flag=1
        elif ppd_old <= ppd:
            forward_flag=0
        ppd_old=ppd
        print('vitual_wall_flag')
        print(vitual_wall_flag)
        if flag==1:
            if ppd>1000:
                vitual_wall_flag=1
            if ppd<450 and round_flag > 0:
                light['R'] = _LIGHT_MAX/2+0.5*(300.0-ppd)*(line_theta-current_theta)
                print('light')
                print(light['R'])
            if ppd<300 and vitual_wall_flag==1:
                if np.abs(current_theta-line_theta)>10:
                    print(current_theta)
                    print(line_theta)
                    state[0] = STATES['collision']
                elif np.abs(current_theta-line_theta)<10:
                    vitual_wall_flag=0
                #round_flag = round_flag - 1
            elif ppd<150 and forward_flag==1:
                bump['any'] = 1
                forward_flag=0
                #round_flag = 2

        # follow the wall
        if state[0] == STATES['follow_wall']:
            path = pid(light) # get path from pid controller
            if light['RF'] > _LIGHT_MAX/10: # if obstacle in front
                path = [[ 60,-60, 0.1, 'left']] # turn left until no obstacle
            if bump['any']: # if hit an obstacle, get back a bit
                path = [[-60,-60, 0.5, 'back']]
                state[0] = STATES['collision']
            continue

        # if hit an obstacle, after get back, turn left a bit
        if state[0] == STATES['collision']:
            path = [[ 60,-60, 0.7, 'left']]
            state[0] = STATES['follow_wall']
            continue


    # quit from follow wall
    path = [[ 0,0, 0.1, 'stop']] # stop robot, prevent the robot keep moving under previous 'path'
    for lft, rht, dt, s in path:
            bot.digit_led_ascii(s)
            bot.drive_direct(lft, rht)
            time.sleep(dt)



# v: forward velocity
# d: distance to the wall
def pid(light,v=60,d=2):
    P = d - _LIGHT_MAX/(1+light['R'])
    k = min(max(2.5*P,-.5),.5)
    if light['R'] < 80:
        k = min(max(2.5*P,-.7),.7)
    if k > 0:
        path = [[ int(v*(1+k)), int(v*(1-k)), 0.1, 'pidL']]
    else:
        path = [[ int(v*(1+k)), int(v*(1-k)), 0.1, 'pidR']]
    return path


#find perpendicular point for (a,b) on (x1,y1)->(x2,y2)
def point_cal(x1,y1,x2,y2,a,b):
    #point (c,d)
    x1=float(x1[0])
    y1=float(y1[0])
    x2=float(x2[0])
    y2=float(y2[0])
    a=float(a)
    b=float(b)

    c=(y2-y1)*(y2-y1)*(x1-x2)/( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) ) *( y1/(y2-y1) -x1/(x2-x1) -b/(y2-y1) -(x2-x1)/(y2-y1)/(y2-y1)*a )
    d=b-(x2-x1)/(y2-y1)*(c-a)
    #if the point(c,d) is between (x1,y1)->(x2,y2)
    alpha=(c-x1)/(x2-x1)
    if alpha>1:
        flag = 0
    elif alpha<0:
        flag = 0
    else:
        flag = 1
    #distance from (a,b) to (x1,y1)->(x2,y2)
    ppd=(a-c)*(a-c)+(b-d)*(b-d)
    ppd=np.sqrt(ppd)

    return c,d,ppd,flag

if __name__ == "__main__":

    ROBOT_DEVICE, LIDAR_DEVICE = find_device()

    # map init
    MAP_SIZE_METERS = 20 # map resolution adjustment
    MAP_SIZE_PIXELS = 300 # map size

    mapbytes = bytearray(MAP_SIZE_PIXELS**2) # map
    posbytes = bytearray(MAP_SIZE_PIXELS**2) # robot position

    # bluetooth init
    hostMACAddress = 'B8:27:EB:C4:D0:94'
    #hostMACAddress = '24:FD:52:8C:04:31'#wang pc


    # robot init
    STATES = {'find_wall':0, 'reach_wall':1, 'follow_wall':2, 'collision': 3, 'stop': 4, 'remote': 5}
    state = [STATES['stop']] # init robot state
    odometry = [0., 0., 0.] # encoder, angle, dt

    command = [b'w'] # wait

    thread_bluetooth = threading.Thread(target=run_bluetooth, args=[hostMACAddress, mapbytes, posbytes, command, state, STATES])
    thread_slam = threading.Thread(target=run_slam, args=[LIDAR_DEVICE, MAP_SIZE_PIXELS, MAP_SIZE_METERS, mapbytes, posbytes, odometry, command, state, STATES])
    thread_robot = threading.Thread(target=run_robot, args=[ROBOT_DEVICE, odometry, command, state, STATES])

    thread_bluetooth.start()
    thread_slam.start()
    thread_robot.start()

    thread_bluetooth.join()
    thread_slam.join()
    thread_robot.join()