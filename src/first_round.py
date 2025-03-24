#imported libraries that are necesary for the project
import cv2         # imports openCV
import numpy as np # math. and sci. computing library
from picamera import PiCamera # camera and it basic functions
from time import sleep # time measurments
import RPi.GPIO as GPIO 
from mpu6050 import mpu6050 #for the gyro
from rplidar import RPLidar # for the lidar
import matplotlib.pyplot as plt 
from scipy.integrate import odeint
class PD_lidar:
    kp= 1 #Preliminary values, eligible for testing
    kd= 1 #Preliminary values, eligible for testing
    measurment = 1 #the measurment from the gyro, will be setup later
    setpoint = 0 #lidar vrijednost srednja a za gyro 0 
    def __init__(self, kp, kd, setpoint=0.0):
        self.kp = kp
        self.kd = kd
        self.setpoint = setpoint
        self.last_error = 0.0

    def update(self, measurement, dt): 
        error = self.setpoint - measurement
        derivative = last_error-error
        output = self.kp * error + self.kd * derivative
        self.last_error = error
        return output
class PD_Gyro:
    kp= 1 #Preliminary values, eligible for testing
    kd= 1 #Preliminary values, eligible for testing
    measurment = 1 #the measurment from the gyro, will be setup later
    setpoint = 0 #lidar vrijednost srednja a za gyro 0
    """
    this represents the usual pd code
    error=setpoint-measurment
    derivative=last_error-error
    corection=kp*error+kd*derivative
    last_error=error
    """
    def __init__(self, kp, kd, setpoint=0.0):
        self.kp = kp
        self.kd = kd
        self.setpoint = setpoint
        self.last_error = 0.0

    def update(self, measurement, dt):
        error = self.setpoint - measurement
        derivative = last_error-error
        output = self.kp * error + self.kd * derivative
        self.last_error = error
        return output
#for our color detection we are using the cv2 and numpy libraries as they contain a lot of usefull built in functions   
camera = PiCamera()
# sets up our variables that will be used in the function and returned
detected_color = None
detected_contour = None
#frame represents the input from the camera
#area_threshold is our set value that is taken as the minimum needed for the recognition of the object, it is a preliminary value and its eligible for testing
def colors(frame, area_threshold=500): 
    hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #coverts the frame into HSV color (hue, saturation and value)
    #sets the HSV values for green
    lower_green=np.array([36, 100, 100]) #Preliminary values, eligible for testing
    upper_green=np.array([86, 255, 255])  #Preliminary values, eligible for testing
    green=cv2.inRange(hsv, lower_green, upper_green)
    
    #values for red
    lower_red1 = np.array([0, 100, 100]) #Preliminary values, eligible for testing
    upper_red1 = np.array([10, 255, 255]) #Preliminary values, eligible for testing
    red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    lower_red2 = np.array([160, 100, 100])  #Preliminary values, eligible for testing
    upper_red2 = np.array([180, 255, 255])  #Preliminary values, eligible for testing
    red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    # Combine the two reds
    red = cv2.bitwise_or(red1, red2)
    
    # Find contours in the green
    contours_green, hierarchy = cv2.findContours(green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Find contours in the red
    contours_red, hierarchy = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    
    # Process green contours: choose the largest contour if it meets the area threshold
    if contours_green:
        largest_green = max(contours_green, key=cv2.contourArea)
        if cv2.contourArea(largest_green) > area_threshold:
            detected_color = "green"
            detected_contour = largest_green

    # Process red contours: choose the largest contour if it meets the area threshold
    if contours_red:
        largest_red = max(contours_red, key=cv2.contourArea)
        if cv2.contourArea(largest_red) > area_threshold:
            # In this example, if both colors are detected, red takes priority.
            detected_color = "red"
            detected_contour = largest_red
    # taking values from the image and checking for red and green, that is for where their corners are
    redb=Image.fromarray(red)
    greenb=Image.fromarray(green) 
    red_bbox=redb.getbbox()
    green_bbox=greenb.getbbox()
    # checking if we actually have any value and then puting a frame over our shape for visuals while testing
    if red_bbox is not None:
        x1, y1, x2, y2 = red_bbox
        frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
    cv2.imshow('frame', frame)
    elif green_bbox is not None:
        x1, y1, x2, y2 = green_bbox
        frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
    cv2.imshow('frame', frame)
    
    return detected_color, detected_contour, green, red

def main():
    #color
    ob1, ob2, ob3, ob4, ob5, ob6 = None
    cap=cv2.VideoCapture(0)
    while True:
        ret, frame=cap.read()
        if not ret: 
            break
        detected_color, contuor, green, red = detect_color(frame)
        #the logic of the code will go in the rest of the while loop
    cap.release()
    cv2.destroyAllWindows()
    #color
