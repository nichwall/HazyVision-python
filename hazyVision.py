import numpy as np
import cv2 as cv
import socket
import time

def draw_static(img, connected):
    ''' Draw the image and boxes. '''
    bg = np.zeros((img.shape[0], WIDTH_PX, 3), dtype=np.uint8)
    bg[:,X_OFFSET:X_OFFSET+WEBCAM_WIDTH_PX,:] = img
    cv.rectangle(bg, (search[0]+X_OFFSET,search[1]+CONNECTED_BORDER), ((search[2]-search[0])/2+X_OFFSET,search[3]+CONNECTED_BORDER), (255,0,0), 5)
    cv.rectangle(bg, (WIDTH_PX-(search[0]+X_OFFSET),search[1]+CONNECTED_BORDER),(WIDTH_PX-((search[2]-search[0])/2+X_OFFSET),search[3]+CONNECTED_BORDER), (255,0,0), 5)
    #cv.rectangle(bg, ((search[2]-search[0])/2+X_OFFSET,search[1]+CONNECTED_BORDER), ((search[2])+X_OFFSET,search[3]+CONNECTED_BORDER), (255,0,0), 5)
    if connected:
        cv.rectangle(bg, (0, 0), (bg.shape[1]-1, bg.shape[0]-1), (0, 255, 0), CONNECTED_BORDER)
    else:
        cv.rectangle(bg, (0, 0), (bg.shape[1]-1, bg.shape[0]-1), (0, 0, 255), CONNECTED_BORDER)
    return bg

# Dimensions of the webcam image (it will be resized to this size)
WEBCAM_WIDTH_PX = 640
WEBCAM_HEIGHT_PX = 360

# Width of the entire widget
WIDTH_PX = 1000

X_OFFSET = (WIDTH_PX - WEBCAM_WIDTH_PX)/2

# Connection stuff
HOST, PORT = "10.4.18.2",1180

# This is the rate at which we will send updates to the cRIO.
UPDATE_RATE_HZ = 40.0
PERIOD = (1.0 / UPDATE_RATE_HZ) * 1000.0
#PERIOD = 1

# Constants for drawing.
BOX_BORDER = 3
CONNECTED_BORDER = 15

WINDOW_NAME = "HazyVision"

file = open("config.txt","r")
readed = file.read().split("\n")
file.close()
redHSV = readed[0].split(" ")
greenHSV = readed[1].split(" ")

lowRed = []
highRed = []
lowGreen = []
highGreen = []

# search box = [x,y,x+w,y+h]
search = [80,45,560,315]

errorMargin = 35
for i in range(3):
    lowRed.append(eval(redHSV[i])-errorMargin)
    highRed.append(eval(redHSV[i])+errorMargin)
    lowGreen.append(eval(greenHSV[i])-errorMargin)
    highGreen.append(eval(greenHSV[i])+errorMargin)

RED_LOW = np.array(lowRed)
RED_HIGH = np.array(highRed)
GREEN_LOW = np.array(lowGreen)
GREEN_HIGH = np.array(highGreen)

redRes = 0
greenRes = 0

def contours(redIm, greenIm):
    gGray = cv.cvtColor(cv.cvtColor(redIm,cv.COLOR_HSV2BGR),cv.COLOR_BGR2GRAY)
    rGray = cv.cvtColor(cv.cvtColor(greenIm,cv.COLOR_HSV2BGR),cv.COLOR_BGR2GRAY)
    rContours,rHierachy = cv.findContours(rGray,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    gContours,gHierachy = cv.findContours(gGray,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)    
    rOut = []
    rLargestArea = 0
    largestAreaInd = -1
    minXSearch = search[0]+X_OFFSET
    minYSearch = search[1]+CONNECTED_BORDER
    maxXSearch = search[2]+X_OFFSET
    maxYSearch = search[3]+CONNECTED_BORDER
    for i in range(len(rContours)):
        if cv.contourArea(rContours[i])>20:
            if rHierachy[0][i][3]==-1:
                x,y,w,h = cv.boundingRect(rContours[i])
                if w*h>rLargestArea and x>=minXSearch and w+x<=maxXSearch and y>=minYSearch and h+y<=maxYSearch:
                    rLargestArea=w*h
                    largestAreaInd=i
    if len(rContours)>0 and rLargestArea<15000 and rLargestArea>300:
        rOut.append(rContours[largestAreaInd])

    gOut = []
    gLargestArea = 0
    largestAreaInd = -1
    for i in range(len(gContours)):
        if cv.contourArea(gContours[i])>20:
            if gHierachy[0][i][3]==-1:
                x,y,w,h = cv.boundingRect(gContours[i])
                if w*h>gLargestArea and x>=minXSearch and x+w<=maxXSearch and y>=minYSearch and y+h<=maxYSearch:
                    gLargestArea=w*h
                    largestAreaInd=i
    if len(gContours)>0 and gLargestArea<15000 and gLargestArea>300:
        gOut.append(gContours[largestAreaInd])

    return rOut,gOut

def finder(bg):
    hsv = cv.cvtColor(bg, cv.COLOR_BGR2HSV)
    rMask = cv.inRange(hsv, RED_LOW, RED_HIGH)
    gMask = cv.inRange(hsv, GREEN_LOW, GREEN_HIGH)
    redRes = cv.bitwise_and(bg,bg,mask=rMask)
    greenRes = cv.bitwise_and(bg,bg,mask=gMask)
    red,green = contours(redRes,greenRes)
    return red,green

def get_time_millis():
    ''' Get the current time in milliseconds. '''
    return int(round(time.time() * 1000))

def getSpeed(x):
    leftBound = search[0]+X_OFFSET
    rightBound = (search[2]-search[0])/2+X_OFFSET
    centerLine = WIDTH_PX/2
    toLeft = 1
    if (x > centerLine):
        x = centerLine-(x-centerLine)
        toLeft = 0
    x -= 260
    if (x>0 and x < 420):
        return toLeft,int(round(0.00000742857*x**3-0.00360679*x**2+0.580664*x,0))
    return 0,0

def main():
    cv.namedWindow(WINDOW_NAME,1)
    
    capture = cv.VideoCapture(0)
    driveRight = False
    driveLeft  = False
    leftStick  = False
    rightStick = False
    connected = False
    last_t = get_time_millis()

    keyPress = 0
    while keyPress!=27:
        has_frame, img = capture.read()
        if not has_frame:
                time.sleep(0.05)
                continue
        small_img = cv.flip(cv.resize(img,(WEBCAM_WIDTH_PX, WEBCAM_HEIGHT_PX)),1)

        #draw
        bg = draw_static(small_img, False)
        red,green = finder(bg)
        boxCenterX = 0
        boxCenterY = 0
        driveRight = False
        driveLeft  = False
        leftStick  = False
        rightStick = False
        if len(red)>0:
            x,y,w,h = cv.boundingRect(red[0])
            boxCenterX = x+w/2
            boxCenterY = y+h/2
            cv.rectangle(bg,(x,y),(x+w,y+h),(0,255,0),2)
            leftStick = True
        if len(green)>0:
            x,y,w,h = cv.boundingRect(green[0])
            boxCenterX = x+w/2
            boxCenterY = y+h/2
            cv.rectangle(bg,(x,y),(x+w,y+h),(0,0,255),2)
            rightStick = True
        cv.rectangle(bg,(search[0]+X_OFFSET,search[1]+CONNECTED_BORDER), (search[2]+X_OFFSET, search[3]+CONNECTED_BORDER), (255,0,0), 2)

        cv.imshow(WINDOW_NAME,bg)
        keyPress = cv.waitKey(10) & 255

        # To send data to robot
        # It will follow the format (as the bits of the number being the booleans
        # 0x32: isRed
        # 0x64: isGreen
        # 0x128: driveRight
        
        # 0x1:   speed 1
        # 0x2:  speed 2
        # 0x4:  speed 3
        # 0x8:  speed 4
        # 0x16: speed 5
        isRed = 0
        isGreen = 0
        if leftStick:
            isRed = 1
        if rightStick:
            isGreen = 1
        isLeft, speed = getSpeed(boxCenterX)
        speedStr = '{0:08b}'.format(speed)
        speed1 = int(speedStr[7])
        speed2 = int(speedStr[6])
        speed3 = int(speedStr[5])
        speed4 = int(speedStr[4])
        speed5 = int(speedStr[3])
        cur_time = get_time_millis()
        # Try to connect to the robot on open or disconnect
        if last_t+PERIOD <= cur_time:
            if not connected:
                try:
                    # Open a socket with the cRIO.
                    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
                    # This is a pretty aggressive timeout...we want to reconnect automatically
                    # if we are disconnected.
                    s.settimeout(.1)
                    s.connect((HOST, PORT))
                    print "connected!"
                except:
                    print "failed to reconnect"
            try:    
                # Send one byte to the cRIO:
                write_bytes = bytearray()
                speed1 = getSpeed(boxCenterX, driveRight)
                v = (speed1 << 0) | (speed2 << 1) | (speed3 << 2) | (speed4 << 3) | (speed5 << 4) | (isRed << 5) | (isGreen << 6) | (isLeft << 7)
                write_bytes.append(v)
                s.send(write_bytes)
                connected = True
                last_t = cur_time
            except:
                print "Could not send data to robot"
                connected = False
    cv.destroyAllWindows()
main()
