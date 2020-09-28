#this is a basic wall follow program that hugs a wall from the right side

import turtle
from pycreate2 import Create2
import time
from pynput import keyboard
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db


#Firebase setup_____________________________________________________________________________
cred = credentials.Certificate('firebase-sdk.json')

firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://hcesc-app.firebaseio.com/'
})

# Create and set up screen and turtle, which is an API where for drawing on a screen________________
t = turtle
t.setup(1000, 1000)
t.Screen()
t.title("Turtle Drawing Program - by Vasudev Ram")
t.showturtle()
#_______________________________________________________________

def pid(sensor,ref1,ref2):
    # it is 'if' statement by now, pid may not work since the sensor is kind of highly unlinear, but you can try PI or P for better performance

    light=sensor.light_bumper_right
    
    if(sensor.light_bumper_right==0):
        light=1
    P=ref1/light
        
    if(P<1):
        L_motor=45
        R_motor=15
        path=[[ L_motor, R_motor, 0.1, 'for']]
        return path
    else:
        t.right(1.9)
        L_motor=15
        R_motor=45
        path=[[ L_motor, R_motor, 0.1, 'for']]
        return path

    return path


if __name__ == "__main__":

    port = '/dev/ttyUSB0' #port depends on which usb port on the rpi the serial cable is plugged into
    baud = {
        'default': 115200,
        'alt': 19200  # shouldn't need this unless you accidentally set it to this
    }
    bot = Create2(port=port, baud=baud['default'])
    bot.start()
    bot.safe()
    bot.full()

    #plays a song on start
    print('Starting ...')
    song = [59, 64, 62, 32, 69, 96, 67, 64, 62, 32, 60, 96, 59, 64, 59, 32, 59, 32, 60, 32, 62, 32, 64, 96, 62, 96]
    song = [76, 16, 76, 16, 76, 32, 76, 16, 76, 16, 76, 32, 76, 16, 79, 16, 72, 16, 74, 16, 76, 32, 77, 16, 77, 16, 77, 16, 77, 32, 77, 16]
    song = [76, 12, 76, 12, 20, 12, 76, 12, 20, 12, 72, 12, 76, 12, 20, 12, 79, 12, 20, 36, 67, 12, 20, 36]
    song = [72, 12, 20, 24, 67, 12, 20, 24, 64, 24, 69, 16, 71, 16, 69, 16, 68, 24, 70, 24, 68, 24, 67, 12, 65, 12, 67, 48]

    print(">> song len: ", len(song)//2)

    # song number can be 0-3
    song_num = 3
    bot.createSong(song_num, song)
    time.sleep(0.1)
    how_long = bot.playSong(song_num)

    # The song will run in the back ground, don't interrupt it
    # how_long is the time in secods for it to finish
    print('Sleep for:', how_long)
    time.sleep(how_long)

    bot.safe()
    cnt = 0
    init_flag=0
    coll_flag=0
    lightmax=0
    
    #firebase link__________________________________________________
    ref = db.reference('/')
    count = 0
    while True:
        
        xcoord = turtle.xcor()
        ycoord = turtle.ycor()
        print(xcoord, ycoord)
        ref.set({ #updates firebase coordinates to current coordinate
            'Coordinates':{
                'current_coord':{
                    'x':xcoord,
                    'y':ycoord
                    }
                }
            })
        
        # Packet 100 contains all sensor data.
        sensor = bot.get_sensors()

        if cnt%20 == 0:
            print("[L ] [LF] [LC] [CR] [RF] [ R]")
            #print(sensor)
        data_L=sensor.light_bumper_left
        data_LF=sensor.light_bumper_front_left
        data_LC=sensor.light_bumper_center_left
        data_CR=sensor.light_bumper_center_right
        data_RF=sensor.light_bumper_front_right
        data_R=sensor.light_bumper_right
#         print(f"{sensor.light_bumper_left:4} {sensor.light_bumper_front_left:4} {sensor.light_bumper_center_left:4} {sensor.light_bumper_center_right:4} {sensor.light_bumper_front_right:4} {sensor.light_bumper_right:4}")
#         print({sensor.light_bumper_left})
#         print({sensor.bumps_wheeldrops.bump_left})
#         print({sensor.bumps_wheeldrops.bump_right})
        if(sensor.light_bumper_left<0 ):
            break
        if(sensor.light_bumper_left>4096 ):
            break

        #time.sleep(.01)
#init_set

        ref1=2000
        ref2=100
        #wall follow
        path=pid(sensor,ref1,ref2)

        #collision avoidance
        cornerTurn = False
        if(sensor.light_bumper_front_right>60):
            cornerTurn = True
            path=[[60,-60,0.1,'left']]
            t.left(4)
            

#drive
        
        for lft, rht, dt, s in path:
            print(s)
            bot.digit_led_ascii(s)
            bot.drive_direct(lft, rht)
            if cornerTurn:
                t.left(4)
                continue
            else:
                t.forward(dt*30)
            time.sleep(dt)
        #print(lightmax)


        cnt += 1#!/usr/bin/env python3
        
        

