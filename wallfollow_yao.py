#!/usr/bin/env python3



from pycreate2 import Create2
import numpy as np
import time
import turtle
import threading
import sys, select


# all robot states
STATES = {'find_wall':0, 'reach_wall':1, 'follow_wall':2, 'collision': 3, 'stop': 4, 'remote': 5}
state = STATES['find_wall'] # init robot state
pos = np.zeros((3,)) # x, y, angle
remote = [0,0,0] # forward, left, right


# v: forward velocity
# d: distance to the wall
def pid(light,v=40,d=2):
    P = d - 4096/(1+light['R'])
    k = min(max(2.5*P,-.5),.5)
    if light['R'] < 80:
        k = min(max(2.5*P,-.7),.7)
    if k > 0:
        path = [[ int(v*(1+k)), int(v*(1-k)), 0.1, 'pidL']]
    else:
        path = [[ int(v*(1+k)), int(v*(1-k)), 0.1, 'pidR']]
    return path


# return true if a key is waiting to read
def kbhit():
    dr,__,__ = select.select([sys.stdin],[],[],0)
    return dr != []




def find_wall(robot):
    global state
    path = [[  0,  0, 0.1, 'strt']] # init path

    # find wall if robot is not issued to remote control or stop
    # terminates when a wall is reached
    while state not in [STATES['stop'], STATES['remote'], STATES['follow_wall']]:
        light, bump, __ = bot.get_sensors_uvbot() # read sensor
        print(light, bump, state, path[0][:2])

        # robot move on 'path'
        for lft, rht, dt, s in path:
            bot.digit_led_ascii(s)
            bot.drive_direct(lft, rht)
            time.sleep(dt)

        # find a wall
        if state == STATES['find_wall']:
            if not bump['any']: # if not reach a wall, go forward
                path = [[ 20, 20, 0.1, 'forw']]
            else: # if reached a wall, backward a bit
                state = STATES['reach_wall']
                path = [[-20,-20, 0.7, 'back']]
                print('reach wall')
            continue

        # rotate to a good direction for wall following
        if state == STATES['reach_wall']:
            if light['R'] < 400: # rotate until right sensor find a wall
                path = [[40, -40, 0.1, 'left']]
            else: # pasue 1 sec. before follow the wall
                path = [[  0,  0,   1, 'paus']]
                state = STATES['follow_wall']
                print('follow wall')
            continue


def wall_follow_map(robot):
    global state

    path = [[  0,  0, 0.1, 'strt']] # init path

    __, __, encoder = bot.get_sensors_uvbot() # read sensor
    encoder_last = np.array([encoder['L'], encoder['R']]) # remember initial encoder reading

    # follow wall if robot is not issued to remote control or stop or find a wall
    while state not in [STATES['stop'], STATES['find_wall'], STATES['remote']]:
        light, bump, encoder = bot.get_sensors_uvbot() # read from sensor
        print(light, path[0][:2], encoder_last)

        # robot move on 'path'
        for lft, rht, dt, s in path:
            bot.digit_led_ascii(s)
            bot.drive_direct(lft, rht)
            time.sleep(dt)

        # calculate robot position and orientation
        encoder_new = np.array([encoder['L'], encoder['R']]) - encoder_last # encoder incremental
        encoder_new += (encoder_new > 1e4) * (-65536) + (encoder_new < -1e4) * 65536 # remove overflow

        encoder_last = np.array([encoder['L'], encoder['R']]) # update restored encoder reading
        encoder_new_mm = encoder_new * np.pi * 72.0 / 508.8 # encoder incremental in mm
        ang_new = (encoder_new_mm[1] - encoder_new_mm[0]) / 235.0  # orientation incremental in rad
        pos[2] += ang_new # update current orientation

        if ang_new:
            pos[:2] = pos[:2] + np.mean(encoder_new_mm) * (np.sin(ang_new)/ang_new * np.array([np.cos(pos[2]), np.sin(pos[2])]) + (1-np.cos(ang_new))/ang_new * np.array([np.sin(pos[2]), -np.cos(pos[2])]))
        else:
            pos[:2] = pos[:2] + np.mean(encoder_new_mm)*np.array([np.cos(pos[2]), np.sin(pos[2])]) # update current position



        # follow the wall
        if state == STATES['follow_wall']:
            path = pid(light) # get path from pid controller
            if light['RF'] > 400: # if obstacle in front
                path = [[ 60,-60, 0.1, 'left']] # turn left until no obstacle
            if bump['any']: # if hit an obstacle, get back a bit
                path = [[-60,-60, 0.5, 'back']]
                state = STATES['collision']
            continue

        # if hit an obstacle, after get back, turn left a bit
        if state == STATES['collision']:
            path = [[ 60,-60, 0.5, 'left']]
            state = STATES['follow_wall']
            continue

    # quit from wall follow
    path = [[ 0,0, 0.1, 'stop']] # stop robot, prevent the robot keep moving under previous 'path'
    for lft, rht, dt, s in path:
            bot.digit_led_ascii(s)
            bot.drive_direct(lft, rht)
            time.sleep(dt)


def remote_control():
    global remote

    print('remote control')
    path = [[ 0,0, 0.1, 'paus']]

    while state not in [STATES['stop'], STATES['find_wall']]:

        if remote == [0,0,0]:
            path = [[ 0,0, 0.1, 'paus']]
        elif remote == [1,0,0]:
            path = [[ 40,40, 0.1, 'forw']]
        elif remote == [0,1,0]:
            path = [[ 20,-20, 0.1, 'left']]
        elif remote == [0,0,1]:
            path = [[-20, 20, 0.1, 'righ']]

        for lft, rht, dt, s in path:
            bot.digit_led_ascii(s)
            bot.drive_direct(lft, rht)
            time.sleep(dt)

    print ('remote control end')


def run_robot():

    global state
    global pos # calculate pos
    global STATES
    global remote

    # run the robot until stop is issued
    while state is not STATES['stop']:
        if state is STATES['remote']: # if in remote control mode, do remote control
            remote_control()
            continue

        if state is STATES['find_wall']: # if in find wall mode, go find the wall
            find_wall(bot)
            continue

        wall_follow_map(bot) # o.w. follow the wall

    print('Thread_robot end')

def run_ui():

    global state
    global pos # use pos calculated in robot thread
    global STATES
    global remote

    turtle.setup(600,400) # setup canvas for map
    turtle.Screen()
    turtle.showturtle()
    turtle.pd() # pen dowm
    last_pos = np.copy(pos) # remember last position

    # run the ui until stop is issued
    while state is not STATES['stop']:
        if last_pos is not pos: # if robot went to a new position
            turtle.setpos(pos[0]/20.,pos[1]/20.) # plot the new position
        if kbhit(): # is key is pressed
            key = sys.stdin.read(1) # read the key (you need to press enter)
            if key is 'q': # if 'q' is pressed, stop the robot
                state = STATES['stop']
            elif key is 'r': # if 'r' is pressed, go remote control
                state = STATES['remote']
            elif key is 'f': # if 'f' is pressed, go find wall
                state = STATES['find_wall']

            if state is STATES['remote']:
                if key is 'w':
                    remote = [1,0,0]
                elif key is 's':
                    remote = [0,0,0]
                elif key is 'a':
                    remote = [0,1,0]
                elif key is 'd':
                    remote = [0,0,1]

    print('Thread_ui end')


if __name__ == "__main__":

    port = '/dev/ttyUSB0'
    baud = {'default': 115200, 'alt': 19200} # shouldn't need 'alt' unless you accidentally set it to this
    bot = Create2(port=port, baud=baud['default'])

    bot.start()
    bot.safe()

    #########################################################################################################
    # play start music
    print('Starting ...')

    song = [76, 12, 76, 12, 20, 12, 76, 12, 20, 12, 72, 12, 76, 12, 20, 12, 79, 12, 20, 36, 67, 12, 20, 36]
    song_num = 3  # song number can be 0-3
    bot.createSong(song_num, song)
    time.sleep(0.1)
    how_long = bot.playSong(song_num) # how_long is the time in secods for it to finish
    time.sleep(how_long)

    print('Wait ', how_long, ' for music')
    #########################################################################################################

    thread_robot = threading.Thread(target=run_robot)
    thread_ui = threading.Thread(target=run_ui)

    thread_ui.start()
    thread_robot.start()

    thread_robot.join()
    thread_ui.join()