#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
  _\
  \
O O-O
 O O
  O

Raspberry Potter
Version 0.1.5

Use your own wand or your interactive Harry Potter wands to control the IoT.

Updated for OpenCV 3.2
If you have an older version of OpenCV installed, please uninstall fully (check your cv2 version in python) and then install OpenCV following the guide here (but using version 3.2):
https://imaginghub.com/projects/144-installing-opencv-3-on-raspberry-pi-3/documentation

Copyright (c) 2015-2017 Sean O'Brien.  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''
import io
import sys
import numpy as np
import cv2
import threading
import math
import time

#FindWand is called to find all potential wands in a scene.  These are then tracked as points for movement.  The scene is reset every 3 seconds.
def FindNewPoints():
    global old_frame,old_gray,p0,ig
    
    # Get image from camera
    old_frame = getFrame(cam)
    
    # Convert image to greyscale
    old_gray = cv2.cvtColor(old_frame,cv2.COLOR_BGR2GRAY)

    #TODO: trained image recognition
    
    # Get circles in image
    # Function returns array of coordinates of center points and radii in format (x, y, radius)
    p0 = cv2.HoughCircles(old_gray,cv2.HOUGH_GRADIENT,3,100,param1=100,param2=30,minRadius=4,maxRadius=15)
    
    # Reshape array - possibly unnecessary?
    p0.shape = (p0.shape[1], 1, p0.shape[2])
    
    # Strip array of the radius
    p0 = p0[:,:,0:2]
    
    # Create/reset array of gesture data
    ig = [[0] for x in range(20)]
    
    print("Found points")
    return True
    
def TrackWand():
    global old_frame,old_gray,p0,mask,color,ig
    
    # Parameters for image processing
    lk_params = dict( winSize  = (15,15),
                    maxLevel = 2,
                    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    movment_threshold = 80

    color = (0,0,255)
    
    old_frame = getFrame(cam)
    old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)

    # Take first frame and find circles in it
    p0 = cv2.HoughCircles(old_gray,cv2.HOUGH_GRADIENT,3,100,param1=100,param2=30,minRadius=4,maxRadius=15)
    try:
        p0.shape = (p0.shape[1], 1, p0.shape[2])
        p0 = p0[:,:,0:2]
    except:
        print("No points found")
	# Create a mask image for drawing purposes
    mask = np.zeros_like(old_frame)

    while True:
        frame = getFrame(cam)
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        try:
            # calculate optical flow
            p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
            # Select good points
            good_new = p1[st==1]
            good_old = p0[st==1]
            # draw the tracks
            for i,(new,old) in enumerate(zip(good_new,good_old)):
                a,b = new.ravel()
                c,d = old.ravel()
                # only try to detect gesture on highly-rated points (below 15)
                if (i<15):
                    IsGesture(a,b,c,d,i)
                    dist = math.hypot(a - c, b - d)
                    if (dist<movment_threshold):
                        cv2.line(mask, (a,b),(c,d),(0,255,0), 2)
                        cv2.circle(frame,(a,b),5,color,-1)
                        cv2.putText(frame, str(i), (a,b), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255))
        except IndexError:
            print("Index error")
            End()
            break
        except:
            e = sys.exc_info()[0]
            print("TrackWand Error: %s" % e )
            End()
            break
        updateWindow(cv2.add(frame,mask))

        # get next frame
        old_frame = getFrame(cam)

        # Now update the previous frame and previous points
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1,1,2)

#Spell is called to translate a named spell into GPIO or other actions
def Spell(spell):
    #clear all checks
    ig = [[0] for x in range(15)]
    #Invoke IoT (or any other) actions here
    cv2.putText(mask, spell, (5, 25),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,0,0))
    if (spell=="Colovaria"):
        print("GPIO trinket")
        pi.write(trinket_pin,0)
        time.sleep(1)
        pi.write(trinket_pin,1)
    elif (spell=="Lumos"):
        print("GPIO ON")
        pi.write(switch_pin,1)
    elif (spell=="Nox"):
        print("GPIO OFF")
        pi.write(switch_pin,0)
    print("CAST: %s" %spell)

#IsGesture is called to determine whether a gesture is found within tracked points
def IsGesture(a,b,c,d,i):
    print("point: %s" % i)
    #record basic movements - TODO: trained gestures
    if ((a<(c-5))&(abs(b-d)<1)):
        ig[i].append("left")
    elif ((c<(a-5))&(abs(b-d)<1)):
        ig[i].append("right")
    elif ((b<(d-5))&(abs(a-c)<5)):
        ig[i].append("up")
    elif ((d<(b-5))&(abs(a-c)<5)):
        ig[i].append("down")
    #check for gesture patterns in array
    astr = ''.join(map(str, ig[i]))
    if "rightup" in astr:
        print("Spell(\"Lumos\")")
    elif "rightdown" in astr:
        print("Spell(\"Nox\")")
    elif "leftdown" in astr:
        print("Spell(\"Colovaria\")")
    print(astr)

def End():
	cam.release()
	cv2.destroyAllWindows()

# Get the next frame from the camera feed
def getFrame(cam):
    # Read next frame from camera
    returnCode, frame = cam.read()

    # If returnCode is not true, error has occured
    if not returnCode:
        raise Exception("Error reading from camera")

    cv2.flip(frame,1,frame)
    return frame

def updateWindow(img):
    cv2.putText(img, "Press ESC to close.", (5, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255))
    cv2.imshow("Raspberry Potter", img)
    cv2.waitKey(1)

# Starts camera input and runs FindNewPoints
if __name__=="__main__":
    # Create window for displaying picture
    cv2.namedWindow("Raspberry Potter")
    
    # Open camera
    cam = cv2.VideoCapture(0)
    if cam.isOpened():
        print("Camera started")
    
    # Set camera resolution
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
    
    # Start running main loop
    try:
        running = True
        while running:
            if not FindNewPoints():
                running = False
            elif not TrackWand():
                running = False
            elif cv2.getWindowProperty("Raspberry Potter", 0) < 0:
                # Is window still open? If not, stop running
                running = False
    except KeyboardInterrupt:
        pass
    End()
    exit
