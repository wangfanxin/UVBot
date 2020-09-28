import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
from pycreate2 import Create2
import time


#Firebase setup_____________________________________________________________________________
cred = credentials.Certificate('firebase-sdk.json')

firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://hcesc-app.firebaseio.com/'
})


#Robot initialization________________________________________________________________

if __name__ == "__main__":
    port = '/dev/ttyUSB0' #this port depends on which usb port on the r-pi the serial cable is plugged into
    baud = {
        'default': 115200,
        'alt': 19200  # shouldn't need this unless you accidentally set it to this
    }

    bot = Create2(port=port, baud=baud['default'])

    bot.start()

    bot.safe()
    bot.full()

    print('Starting ...')

    bot.safe()
    cnt = 0
    init_flag=0
    coll_flag=0
    lightmax=0

    
#controls___________________________________________________________________
    ref = db.reference('remoteControl')  #reads from database child 'remoteControl'
    while True:
        #retrieve values from firebase database under the child 'remoteControl'
        forwardValue = ref.child('forward').get()
        backwardValue = ref.child('backward').get()
        leftValue = ref.child('left').get()
        rightValue = ref.child('right').get()
                
        #if the value of a given control is 1 then activate it, if 0 then deactivate it        
        if forwardValue == 1:
            bot.drive_direct(60,60)
            print('forward')
        if backwardValue == 1:
            bot.drive_direct(-60,-60)
            print('backward')
        if leftValue == 1:
            bot.drive_direct(40,-40)
            print('left')
        if rightValue == 1:
             bot.drive_direct(-40,40)
             print('right')
        if forwardValue == 0 and backwardValue == 0 and leftValue == 0 and rightValue == 0:
            bot.drive_direct(0,0)
            print('stop')
                

