"""
A simple Python script to send messages to a sever over Bluetooth
using PyBluez (with Python 3).
"""

import bluetooth

serverMACAddress = 'B8:27:EB:C4:D0:94'
port = 1
s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
s.connect((serverMACAddress, port))
while 1:
    text = input()
    text = bytes(text,'utf-8') # Note change to the old (Python 2) raw_input
    if text == b"quit":
        break
    s.send(text)
s.close()
