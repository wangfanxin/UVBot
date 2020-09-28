import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
from pycreate2 import Create2
import time
import turtle
import numpy as np
import math
from timeit import default_timer as timer


#Firebase setup_____________________________________________________________________________
cred = credentials.Certificate('firebase-sdk.json')


firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://hcesc-app.firebaseio.com/'
})

ref = db.reference('pathData')
xValues = []
yValues = []
arrayLength = ref.child('length').get()

# Create and set up screen and turtle______________________________________

t = turtle
t.setup(600, 600)
t.Screen()
t.title("Turtle Drawing Program - by Vasudev Ram")
t.showturtle()

#robot initialization__________________________________________________________
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

    print('Starting ...')

    bot.safe()

#extract data from database and put into arrays___________________
index = 0
for i in range(int(arrayLength)):
    xValues.append(ref.child('xvalues').child('x' + str(index)).get())
    yValues.append(ref.child('yvalues').child('y' + str(index)).get())
    if (index == arrayLength):
        break
    index += 2

#print arrays_________________________________________________________
# print(arrayLength)
# count = 0   
# for i in range(len(xValues)):
#     print('(' + str(xValues[count]) + ',' + str(yValues[count]) + ')')
#     count = count + 1

#robot movement functions_______________________________________________
def driveForward(travelDistance):
    startTime = timer()
    time = travelDistance * 0.05
    endTime = timer()
    #the time it drives depends on the travelDistance variable 
    while (endTime - startTime < time): 
        bot.drive_direct(20,20)
        endTime = timer()
        
    bot.drive_direct(0,0)

def turnLeft(turnAngle):
    startTime = timer()
    time = turnAngle * 0.04
    endTime = timer()
    #the time it turns depends on the variable turnAngle
    while (endTime - startTime < time):
        bot.drive_direct(20, -20)
        endTime = timer()
        
    bot.drive_direct(0,0)

def turnRight(turnAngle):
    startTime = timer()
    time = turnAngle * 0.04
    endTime = timer()
    #the time it turns depends on the variable turnAngle
    while (endTime - startTime < time):
        bot.drive_direct(-20, 20)
        endTime = timer()
        
    bot.drive_direct(0,0)
        

#function to find magnitude of vector________________________________
def magnitude(vector = []):
    return math.sqrt((vector[0]**2) + (vector[1]**2))


#function determining which direction to turn given the current and next vector
#Algorithm:
#1. takes current and next vector as args
# 2. rotate the current vector until it is flush with the positive x axis (vector [1,0])
#    direction of rotation is either CW or CCW depending on if the vector is pointing
#     above or below the x-axis
# 3. rotate the next vector by the same angle in the same direction
# 4. the direction to turn can be determined based on a combination of the vectors' x
#     and y values and whether or not they are positive or negative
#     for example: if currentVector had positive x coord and nextVector has negative y
#     coord then this gives a right turn (draw out for reference)
# 5. return direction to turn (return 1 for right turn, 0 for left turn)
def turnDirection(currentVec = [], nextVec = []):
    xAxis = [1,0]
    dotProduct = np.dot(currentVec, xAxis)
    theta = (math.acos((dotProduct)/(magnitude(currentVec)*magnitude(xAxis))))
    print(theta)
    
    if (currentVec[1] >= 0): #clockwise rotation
        rotatedCurrentVecX = currentVec[0]*math.cos(theta) + currentVec[1]*math.sin(theta)
        rotatedCurrentVecY = -1 * currentVec[0]*math.sin(theta) + currentVec[1]*math.cos(theta)
        rotatedNextVecX = nextVec[0]*math.cos(theta) + nextVec[1]*math.sin(theta)
        rotatedNextVecY = -1*nextVec[0]*math.sin(theta) + nextVec[1]*math.cos(theta)
    else: #counterclockwise rotation
        rotatedCurrentVecX = currentVec[0]*math.cos(theta) + -1*currentVec[1]*math.sin(theta)
        rotatedCurrentVecY = currentVec[0]*math.sin(theta) + currentVec[1]*math.cos(theta)
        rotatedNextVecX = nextVec[0]*math.cos(theta) + -1*nextVec[1]*math.sin(theta)
        rotatedNextVecY = nextVec[0]*math.sin(theta) + nextVec[1]*math.cos(theta)
        
    newVec1 = [rotatedCurrentVecX,rotatedCurrentVecY]
    newVec2 = [rotatedNextVecX, rotatedNextVecY]
    
#     print('rotated vec1: ' + str(newVec1))
#     print('rotated vec2: ' + str(newVec2))

    #determines direction to turn based off of rotated vectors and what direction they point
    if (rotatedCurrentVecX > 0 and rotatedNextVecY > 0):
        print('rotated vec1: ' + str(newVec1) + 'first')
        print('rotated vec2: ' + str(newVec2)+ 'first')
        return 1 #1 = right
    elif (rotatedCurrentVecX < 0 and rotatedNextVecY > 0):
        print('rotated vec1: ' + str(newVec1) + 'second')
        print('rotated vec2: ' + str(newVec2)+ 'second')
        return 0 #0 = left
    elif (rotatedCurrentVecX < 0 and rotatedNextVecY < 0):
        print('rotated vec1: ' + str(newVec1) + 'third')
        print('rotated vec2: ' + str(newVec2)+ 'third')
        return 1
    else:
        print('rotated vec1: ' + str(newVec1) + 'fourth')
        print('rotated vec2: ' + str(newVec2)+ 'fourth')
        return 0
    
#parse data_______________________________________________________
turnAngle = 0;
travelDistance = 0;
vectorCurrent = [0,0]
vectorNext = [0,0]
scalingFactor = 0.4
prevDirection = 0

index = 0
#takes 3 points from the point coordinate arrays to create 2 vectors, current and next
for i in range(len(xValues) - 2):
    firstPoint = [xValues[index],yValues[index]]
    secondPoint = [xValues[index + 1], yValues[index + 1]]
    thirdPoint = [xValues[index + 2], yValues[index + 2]]
            
    vectorCurrent = [secondPoint[0] - firstPoint[0], secondPoint[1] - firstPoint[1]]
    vectorNext = [thirdPoint[0] - secondPoint[0], thirdPoint[1] - secondPoint[1]]
    print('vectorCurrent = ' + str(vectorCurrent))
    print('vectorNext = ' + str(vectorNext))
    
    #turn robot
    if (prevDirection == 0):
        print('turnAngle = ' + str(turnAngle))
        t.left(turnAngle)
        turnLeft(turnAngle)
        print('left')
    else:
        t.right(turnAngle)
        turnRight(turnAngle)
        print('right')
   
   #determine which direction to turn
    prevDirection = turnDirection(vectorCurrent,vectorNext)
    
    vecCurrentMag = magnitude(vectorCurrent)
    vecNextMag = magnitude(vectorNext)
    
    #distance to travel based on magnitude of current vector
    travelDistance = vecCurrentMag
    print('travelDistance = ' + str(travelDistance))
    t.forward(travelDistance * scalingFactor) 
    driveForward(travelDistance) #drive forward
    
    #calculate angle between current and next vector
    dotProduct = np.dot(vectorCurrent, vectorNext)
    turnAngle = (math.acos((dotProduct)/(vecCurrentMag*vecNextMag))) *(180/np.pi)
    print('turnAngle = ' + str(turnAngle))
    
    index = index + 1

#end case______________________________________________________________________________
print('here: ' + str(vectorCurrent) + str(vectorNext))
#print('suh dude' + turnDirection(vectorCurrent,vectorNext))

#edge case to turn and move the final stretch 
if (turnDirection(vectorCurrent,vectorNext) == 0):
    t.left(turnAngle)
    turnLeft(turnAngle)
    print('left')
else:
    t.right(turnAngle)
    turnRight(turnAngle)
    print('right')

travelDistance = magnitude(vectorNext)
print('travelDistance = ' + str(travelDistance))
t.forward(travelDistance * scalingFactor)
driveForward(travelDistance)
    

    

