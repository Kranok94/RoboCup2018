# -*- encoding: UTF-8 -*-

import almath
import time
import argparse
from naoqi import ALProxy

CAMERA_HORIZONTAL_VISION_ANGLE = 60.97 #Degrees
CAMERA_VERTICAL_VISION_ANGLE = 47.64
CAMERA_WIDTH = 1280 # Pixels
CAMERA_HEIGHT = 960

# Returns Head Pitch and Yaw angles in degrees
# TODO, get calculate also sensed value and return the safest one
def getHeadPitchYaw(motionProxy):
    names         = ["HeadPitch", "HeadYaw"]
    useSensors    = False
    commandAngles = motionProxy.getAngles(names, useSensors)
    commandAngles = [x * almath.TO_DEG for x in commandAngles]
    return commandAngles
    
# Returns the difference in degrees between the center of the camera and center of detected object
# 
def getAngleDelta(pos, length, margin, cameraAngle):
    ratio = float(pos)/length # Get coordinate relative to picture size. 0.5 means pixel is at the middle of picture
    if (ratio < 0.5 + margin) and (ratio > 0.5 - margin): # Given coordinate is almost centered in picture
        return 0.0
    else:
        return (ratio - 0.5) * cameraAngle 
        
def moveHeadJoint(motionProxy, name, position):
    names      = [name]
    angleLists = [[position*almath.TO_RAD]]
    timeLists  = [[1.0]]
    motionProxy.angleInterpolationBezier(names, timeLists, angleLists)

def orientHeadTo(motionProxy, deltaX, deltaY):
    # Tresholds that will limit head movement
    yawCenterTresh = 15
    pitchCenterTreshLookDown = 24
    pitchCenterTreshLookUp = -33
    
    yawSideTresh = 40
    pitchSideTreshLookDown = 18
    pitchSideTreshLookUp = -25
    
    headPitchYaw = getHeadPitchYaw(motionProxy)
    headPitch = headPitchYaw[0]
    headYaw = headPitchYaw[1]
    
    # If head is centered
    if ((deltaX + headYaw) < yawCenterTresh) and ((deltaX + headYaw) > (-1 * yawCenterTresh)):
        # Focus first pitch when head is centered
        moveHeadJoint(motionProxy, "HeadYaw", deltaX + headYaw)
        if (deltaY + headPitch) < pitchCenterTreshLookDown: # Are we going down?
            if (deltaY + headPitch) > pitchCenterTreshLookUp: # Is movement between limits
                moveHeadJoint (motionProxy, "HeadPitch", deltaY + headPitch)
            else:
                moveHeadJoint(motionProxy, "HeadPitch", pitchCenterTreshLookUp)
        else : # If yes but out of limit, go to limit
            moveHeadJoint(motionProxy, "HeadPitch", pitchCenterTreshLookDown)
    else: # Head not centered, check bounds, move pitch first
        if((deltaY + headPitch) < pitchSideTreshLookDown):
            if (deltaY + headPitch) > pitchSideTreshLookUp:
                moveHeadJoint (motionProxy, "HeadPitch", deltaY + headPitch)
            else:
                moveHeadJoint(motionProxy, "HeadPitch", pitchSideTreshLookUp)
        else: # If yes but out of limit, go to limit
            moveHeadJoint(motionProxy, "HeadPitch", pitchSideTreshLookDown)
        # Move yaw after yaw for sequrity
        if((deltaX + headYaw) < yawSideTresh):
            if (deltaX + headYaw) > (-1 * yawSideTresh):
                moveHeadJoint(motionProxy, "HeadYaw", deltaX + headYaw)
            else:
                moveHeadJoint(motionProxy, "HeadYaw", -1 * yawSideTresh)
        else: # If yes but out of limit, go to limit
            moveHeadJoint(motionProxy, "HeadYaw", yawSideTresh)
        
def trackBall(motionProxy, x,y, width, height):
    deltaX = getAngleDelta(x, width, 0.05, CAMERA_HORIZONTAL_VISION_ANGLE)
    deltaY = getAngleDelta(y, height, 0.05, CAMERA_VERTICAL_VISION_ANGLE)
    orientHeadTo(motionProxy, deltaX, deltaY)


def getBallCoordinates():
    photoCP = ALProxy('ALPhotoCapture')
    photoCP.setResolution(2)
    photoCP.setPictureFormat('jpg')
    photoCP.takePictures(5,'/home/nao/pythonProjects', 'nao')
    imageOriginal = cv2.imread('nao_4.jpg')  #take the last image (the good one)

    img = cv2.resize(imageOriginal, None, fx=0.8, fy=0.8, interpolation=cv2.INTER_CUBIC)
    height,width = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ball = ball_cascade.detectMultiScale(gray, 1.3, 5) 
    f = 0
    for (x,y,w,h) in ball:
        centerX = x+h
        centerY = y+h
        rad = int(h/2)
        if(f == 0):
            #cv2.rectangle(img,(centerX-rad,y),(centerX+rad,centerY),(0,0,255),2)
            area = 3.1416*rad*rad
            #print ('Width: %d, Height: %d, CenterXY: (%d,%d), Area: %d Duration: %5.2f' %(width, height, centerX, y+rad, area, end-start))         
            return centerX,y+rad,area
        f = 1

        
def main(robotIP, PORT = 9559):
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    motionProxy.setStiffnesses("Head", 1.0)
    
    while True:
        x,y,area = getBallCoordinates()
        '''
        numX = raw_input("X: ")
        X = int(numX)
        numY = raw_input("Y: ")
        Y = int(numY)
        '''
        trackBall(motionProxy, x, y, 100, 75)
    motionProxy.setStiffnesses("Head", 0.0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot ip address")
    parser.add_argument("--port", type=int, default=9559,
                        help="Robot port number")

    args = parser.parse_args()
    main(args.ip, args.port)
