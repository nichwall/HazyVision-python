import numpy as np
import cv2 as cv
import socket
import time
from datetime import datetime
import math

def draw_static(img, connected):
    ''' Draw the image and boxes. '''
    bg = np.zeros((img.shape[0], WIDTH_PX, 3), dtype=np.uint8)
    bg[:,X_OFFSET:X_OFFSET+WEBCAM_WIDTH_PX,:] = img
    cv.rectangle(bg, TARGET_UL, TARGET_LR, (0,255,255), BOX_BORDER)
    if connected:
        cv.rectangle(bg, (0, 0), (bg.shape[1]-1, bg.shape[0]-1), (0, 255, 0), CONNECTED_BORDER)
    else:
        cv.rectangle(bg, (0, 0), (bg.shape[1]-1, bg.shape[0]-1), (0, 0, 255), CONNECTED_BORDER)
    return bg

def detect_color(img, box):
    ''' Return the average HSV color of a region in img. '''
    h = np.mean(img[box[0][1]+3:box[1][1]-3, box[0][0]+3:box[1][0]-3, 0])
    s = np.mean(img[box[0][1]+3:box[1][1]-3, box[0][0]+3:box[1][0]-3, 1])
    v = np.mean(img[box[0][1]+3:box[1][1]-3, box[0][0]+3:box[1][0]-3, 2])
    return (h,s,v)

def detect_colors(img):
    ''' Return the average colors for the calibration, left, and right boxes. '''
    target = detect_color(img, (TARGET_UL, TARGET_LR))
    return target

# Dimensions of the webcam image (it will be resized to this size)
WEBCAM_WIDTH_PX = 640
WEBCAM_HEIGHT_PX = 360

# Width of the entire widget
WIDTH_PX = 1000

X_OFFSET = (WIDTH_PX - WEBCAM_WIDTH_PX)/2

# Constants for drawing.
BOX_BORDER = 3
CONNECTED_BORDER = 15

WINDOW_NAME = "Configure values"

# area for targeting box to check
TARGET_UL = (280+X_OFFSET,130)
TARGET_LR = (360+X_OFFSET,195)

def convertToHSV(array):
    a = np.uint8([[array]])
    hsv_a = cv.cvtColor(a,cv.COLOR_BGR2HSV)
    return hsv_a[0][0]

def main():
    print "This will configure the colors to search for"
    print "We will calibrate red, and then green"
    raw_input("Press enter when ready...")
    cv.namedWindow(WINDOW_NAME,1)
    
    capture = cv.VideoCapture(0)
    startTime = datetime.now()
    lastTimeDiff=6
    while True:
        has_frame, img = capture.read()
        if not has_frame:
                time.sleep(0.05)
                continue
        small_img = cv.flip(cv.resize(img,(WEBCAM_WIDTH_PX, WEBCAM_HEIGHT_PX)),1)

        #draw
        bg = draw_static(small_img, False)

        timeDiff = (datetime.now()-startTime).seconds

        cv.imshow(WINDOW_NAME,bg)
        key = cv.waitKey(10) & 255

        if timeDiff!=lastTimeDiff:
            print str(5-timeDiff)+" seconds remaining"

        if key==27 or timeDiff>=5:
                break
        lastTimeDiff = timeDiff
    redVals = convertToHSV(detect_colors(bg))

    print "About to start calibrating green..."
    raw_input("Press enter when ready to begin.")

    startTime = datetime.now()
    lastTimeDiff=6
    while True:
        has_frame, img = capture.read()
        if not has_frame:
                time.sleep(0.05)
                continue
        small_img = cv.flip(cv.resize(img,(WEBCAM_WIDTH_PX, WEBCAM_HEIGHT_PX)),1)

        #draw
        bg = draw_static(small_img, False)

        timeDiff = (datetime.now()-startTime).seconds

        cv.imshow(WINDOW_NAME,bg)
        key = cv.waitKey(10) & 255

        if timeDiff!=lastTimeDiff:
            print str(5-timeDiff)+" seconds remaining"

        if key==27 or timeDiff>=5:
                break
        lastTimeDiff = timeDiff
    greenVals = convertToHSV(detect_colors(bg))

    out = ""
    for i in range(len(redVals)):
        out+=str(redVals[i])+" "
    out+="\n"
    for i in range(len(greenVals)):
        out+=str(greenVals[i])+" "
    file = open("config.txt","w")
    file.write(out)
    file.close()
main()
