#!/usr/bin/env python3



from pycreate2 import Create2
import time
import numpy
import matplotlib.pyplot as plt
'''
def pid(sensor,ref1,ref2):
    # it is 'if' statement by now, pid may not work since the sensor is kind of highly unlinear, but you can try PI or P for better performance
    light=sensor.light_bumper_right
    if(sensor.light_bumper_right==0):
        light=1
    P=ref1/light
    if(P<1):
        L_motor=45
        R_motor=15
    else:
        L_motor=15
        R_motor=45
    path=[[ L_motor, R_motor, 0.1, 'auto']]



    return path
'''
def pid(light,v=40,d=2):
    P = d - 4096/(1+light.light_bumper_right)
    print(P,'P')

    k = min(max(2.5*P,-.5),.5)
    if (1+light.light_bumper_right<80):
        k = min(max(2.5*P,-.7),.7)

    print(k,'k')
    if k > 0:
        path = [[ int(v*(1+k)), int(v*(1-k)), 0.1, 'w L']]
    else:
        path = [[ int(v*(1+k)), int(v*(1-k)), 0.1, 'w R']]
    return path

def robot_init():
    init_flag=0
    coll_flag=0
    while(init_flag==0):

        sensor = bot.get_sensors()
        if(coll_flag==0):
            if(sensor.bumps_wheeldrops.bump_left==False and sensor.bumps_wheeldrops.bump_right==False):
                path= [[ 20, 20, 0.1, 'for']]
            else:
                coll_flag=1
                path=[[ -20, -20, 0.7, 'back']]
        else:
            if(sensor.light_bumper_right<1000):
                path=[[40,-40,0.1,'left']]
            else:
                path=[[0,0,1,'rdy']]
                init_flag=1

        for lft, rht, dt, s in path:
            print(s)
            print('coll:',coll_flag)
            print('init:',init_flag)
            bot.digit_led_ascii(s)
            bot.drive_direct(lft, rht)
            time.sleep(dt)
    return 1

if __name__ == "__main__":

    port = '/dev/ttyUSB0'
    baud = {
        'default': 115200,
        'alt': 19200  # shouldn't need this unless you accidentally set it to this
    }

    bot = Create2(port=port, baud=baud['default'])

    bot.start()

    bot.safe()
    bot.full()

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
    encoder_flag=0
    cum_angle=0.0

    x=numpy.zeros(400)
    y=numpy.zeros(400)
    angle_rec=numpy.zeros(400)
    lightmax=0
    while True:
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
        print(f"{sensor.light_bumper_left:4} {sensor.light_bumper_front_left:4} {sensor.light_bumper_center_left:4} {sensor.light_bumper_center_right:4} {sensor.light_bumper_front_right:4} {sensor.light_bumper_right:4}")
        print({sensor.light_bumper_left})
        print({sensor.bumps_wheeldrops.bump_left})
        print({sensor.bumps_wheeldrops.bump_right})
        if(sensor.light_bumper_left<0 ):
            break
        if(sensor.light_bumper_left>4096 ):
            break

        #time.sleep(.01)

        if (init_flag==0):
            robot_init()
            init_flag=1
        ref1=2000
        ref2=50
        #wall follow
        path=pid(sensor)

        #collision avoidance
        if(sensor.light_bumper_front_right>400):
            path=[[60,-60,0.1,'left']]
        #collision detected
        if(coll_flag==1):
            path=[[60,-60,0.5,'left']]
            coll_flag=0
        if(sensor.bumps_wheeldrops.bump_left or sensor.bumps_wheeldrops.bump_right):
            path=[[-60,-60,0.5,'back']]
            coll_flag=1



        print(path[0][:2])

#drive
        for lft, rht, dt, s in path:
            print(s)
            bot.digit_led_ascii(s)
            bot.drive_direct(lft, rht)
            time.sleep(dt)
        print(lightmax)

#encoder+travel+angle
        if(encoder_flag==0):
            last_left=sensor.encoder_counts_left
            last_right=sensor.encoder_counts_right
            encoder_flag=1



        left_travel=sensor.encoder_counts_left-last_left
        right_travel=sensor.encoder_counts_right-last_right
        last_left=sensor.encoder_counts_left
        last_right=sensor.encoder_counts_right

        if(path[0][0]>0):
            left_sign=1.0
        else:
            left_sign=1.0

        if(path[0][1]>0):
            right_sign=1.0
        else:
            right_sign=1.0


        print(left_sign,right_sign,left_travel,right_travel,last_left,last_right)

        left_mm=(left_travel)*(3.1415926*72.0/508.8)*left_sign
        right_mm=(right_travel)*(3.1415926*72.0/508.8)*right_sign

        turn_angle=(right_mm-left_mm)/235.0 #rad

        cum_angle=turn_angle+cum_angle


        print(left_mm,' ',right_mm)
        print(cum_angle)
        x[cnt+1]=x[cnt]+numpy.cos(cum_angle)*(left_mm+right_mm)/2.0
        y[cnt+1]=y[cnt]+numpy.sin(cum_angle)*(left_mm+right_mm)/2.0
        angle_rec[cnt]=cum_angle

        cnt += 1
        print('cnt:',cnt)
        if(cnt>398):
            path=[[0,0,1,'stop']]
            for lft, rht, dt, s in path:
                print(s)
                bot.digit_led_ascii(s)
                bot.drive_direct(lft, rht)
                time.sleep(dt)

            break
    print(x)
    print(y)

    plt.plot(x,y)
    plt.axis('scaled')
    plt.show()