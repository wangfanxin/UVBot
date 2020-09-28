from pynput import keyboard
from pycreate2 import Create2
import time

#Robot initialization________________________________________________________________

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

    bot.safe()
    cnt = 0
    init_flag=0
    coll_flag=0
    lightmax=0


#Key board controls__________________________________________________________
    def on_press(key):
        
        if key == keyboard.Key.up:
            bot.drive_direct(100,100)
        if key == keyboard.Key.down:
            bot.drive_direct(-100,-100)
        if key == keyboard.Key.left:
            bot.drive_direct(100,-100)
        if key == keyboard.Key.right:
             bot.drive_direct(-100,100)
                

    def on_release(key):
        print('{0} released'.format(key))
        if key == keyboard.Key.up or key == keyboard.Key.down or key == keyboard.Key.left or key == keyboard.Key.right:
            bot.drive_direct(0,0)
    
        

    # Collect events until released
    with keyboard.Listener(on_press=on_press,on_release=on_release) as listener:
        listener.join()

    # ...or, in a non-blocking fashion:
    listener = keyboard.Listener(on_press=on_press,on_release=on_release)
    listener.start()



