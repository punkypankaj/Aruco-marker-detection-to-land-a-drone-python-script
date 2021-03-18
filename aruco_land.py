from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse  

import numpy as np
import cv2
from cv2 import aruco



cap = cv2.VideoCapture(0)




parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=921600, wait_ready=True)




# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print "Basic pre-arm checks"
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print " Waiting for vehicle to initialise..."
    time.sleep(1)
        
  print "Arming motors"
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print " Waiting for arming..."
    time.sleep(1)

  print "Taking off!"
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print " Altitude: ", vehicle.location.global_relative_frame.alt 
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print "Reached target altitude"
      break
    time.sleep(1)

#Takeoff height in meters
arm_and_takeoff(8)

print("Take off complete")

# Hover for 10 seconds
time.sleep(2)


while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)



    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)


    for rejectedPolygons in rejectedImgPoints:
         for points in rejectedPolygons:
            cv2.line(frame_markers, tuple(points[0]), tuple(points[1]), [100, 0, 100])
            cv2.line(frame_markers, tuple(points[2]), tuple(points[1]), [100, 0, 100])
            cv2.line(frame_markers, tuple(points[2]), tuple(points[3]), [100, 0, 100])
            cv2.line(frame_markers, tuple(points[0]), tuple(points[3]), [100, 0, 100])

  #  cv2.imshow('frame_marker',frame_markers)
  #  "53" is the index id of that particular Aruco Marker ( tetsed with 4X4 matrix marker)
    if (ids==53):
       print("Now let's land")
        
       vehicle.mode = VehicleMode("LAND")

       break

cap.release()
cv2.destroyAllWindows()





# Close vehicle object
vehicle.close()

