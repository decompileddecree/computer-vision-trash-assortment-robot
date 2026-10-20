import os
import platform
import time
import cv2
import numpy as np
import serial
import struct
from ultralytics import YOLO
import torch
torch.cuda.set_device(0)

servoSpeed = 0.04  # in seconds
stop = 0
sukses = 0
tolerance = 50

error = (0,0)


integral_error = [0, 0]

robotpause = 1.5
bigservopause = 7

Y_bias = 20


# Flag First Start
maju = 0

# Counter for diferrence routine
# counter == 1 YOLO Scan Detction
# counter == 2 YOLO Move Robot XAxis Based Detction
# counter == 3 Move arm to YOLO saved YAxis
# counter == 4 OpenCV Move Robot to Trash Bin XAxis
# counter == 5 Move arm to Trash bin
counter = 0

start_time = time.time()
previous = 0
interval = 1000  # milliseconds



# Initialize the serial communication
ser = serial.Serial(os.environ.get("ROBOT_SERIAL_PORT", "COM6"), 9600)

# Font for displaying text on the frame
font = cv2.FONT_HERSHEY_SIMPLEX

# Get your screen resolution
screen_width = 1920
screen_height = 1080

# Define Cam Resolution
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

SETPOINT_X = int(CAMERA_WIDTH * 0.59)
# Define Global Variable
received_value = 0

# Initialize the YOLO model
model = YOLO(os.path.join(os.path.dirname(os.path.abspath(__file__)), "best3.pt"))



# Utility function to get the current time in milliseconds
def current_milli_time():
    return round(time.time() * 1000)


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

#global variabel    
last_error = (0, 0)
previous_time = current_milli_time()
last_conf = 100
YOLO_Y_Axis = 0




# function to send serial to arduino
def robot (address, data):
# List address
# addres = data
# 0 = stop = 0
# 1 = Y axis = int -9 - 0 - 9
# 2 = X axis = int -9 - 0 - 9
# 3 = Z axis = int -9 - 0 - 9
# 4 = Low Servo = int 5 - 1900
# 5 = Mid Servo = int 0 - 180
# 6 = Up Servo = int 0 - 180
# 7 = Pump = int = (0 = off) (1 = Suck) (2 = Blow)
# 9 = maju = 0
# 10 = geser = int -9 - 0 - 9
# 11 = PID reset = 0
    
    send = [address, data]
    ser.write(struct.pack('{}h'.format(len(send)), *send))

def resetArduino ():
    ser.close()
    time.sleep(0.1)
    ser.open()
    time.sleep(2)
    ser.close()

# PID constants
kP = 0.08
kI = 0.06
kD = 0.025

clamp_X = 40

PURPLE_COLOR_LOWER = np.array([145, 130, 128])
PURPLE_COLOR_UPPER = np.array([155, 255, 255])

# Flag for Trash Type
# 1 = DAUN              Green
# 2 = KERTAS            Blue
# 3 = LOGAM FERRO       Red
# 4 = LOGAM NONFERRO    White
# 5 = PLASTIK           Yellow
Trash = 0
Trash_label = ''
# Define the color range for detection (HSV format)
# These values are for detecting green color, adjust as necessary
COLOR_LOWER = np.array([0,0,0])
COLOR_UPPER = np.array([0,0,0])
#function to update Trash type in opencv
def trash_type():
    global COLOR_LOWER
    global COLOR_UPPER    
    global Trash_label
    if Trash == 1 :
        #Green
        COLOR_LOWER = np.array([50, 69, 96])
        COLOR_UPPER = np.array([86, 255, 255])
        Trash_label = 'DAUN'

    if Trash == 2 :
        #Blue
        COLOR_LOWER = np.array([99, 140, 50])
        COLOR_UPPER = np.array([140, 255, 255])
        Trash_label = 'KERTAS'

    if Trash == 3 :
        #Red
        COLOR_LOWER = np.array([0, 128, 100])
        COLOR_UPPER = np.array([7, 255, 255])
        Trash_label = 'LOGAM FERRO'

    if Trash == 4 :
        #Red
        COLOR_LOWER = np.array([0, 128, 100])   #0,0,249
        COLOR_UPPER = np.array([7, 255, 255]) #179,1,255
        Trash_label = 'LOGAM NONFERRO'

    if Trash == 5 :
        #Yellow
        COLOR_LOWER = np.array([20, 128, 100])
        COLOR_UPPER = np.array([30, 255, 255])
        Trash_label = 'PLASTIK'

def millis():
    return int((time.time() - start_time) * 1000)


currentmillis = 0
# Global tracking variables
rolling_best_conf = 0
rolling_best_label = None
rolling_best_box = None
frame_counter = 0

# Retry state
conf_cn = 0
shutdown = 0
shutdown_flag = 0
retri = 0
startmillis = millis()



def slowServo(A, B, servo):
    global stop
    #servo = 0 => servoatas
    #servo = 1 => servotengah
    # Move from A to B (either increasing or decreasing)
    if A < B:
        for i in range(A, B + 1):  # inclusive of B
                # Show debug window
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                robot(0, 0)
                stop = 1
                return
            resized_frame = cv2.resize(frame, (screen_width, screen_height))

            cv2.imshow("Detection", resized_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop = 1
            if servo == 0:
                robot(6,int(i))
            elif servo == 1:
                robot(5,int(i))
            time.sleep(servoSpeed)
    elif A > B:
        for i in range(A, B - 1, -1):  # count down to B
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                robot(0, 0)
                stop = 1
                return            
            resized_frame = cv2.resize(frame, (screen_width, screen_height))

            cv2.imshow("Detection", resized_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop = 1            
            if servo == 0:
                robot(6,int(i))
            elif servo == 1:
                robot(5,int(i))
            time.sleep(servoSpeed)


def yoloScan():
    global Trash, counter, shutdown, retri, conf_cn, previous_time, last_error, sukses, tolerance, shutdown_flag
    global rolling_best_conf, rolling_best_label, rolling_best_box, frame_counter, startmillis
    tolerance = 50

    CONFIDENCE_THRESHOLD = 0.60
    REQUIRED_FRAMES = 10  # How many frames we collect before confirming
    TRASH_LABELS = {
        'DAUN': 1,
        'KERTAS': 2,
        'LOGAM FERRO': 3,
        'LOGAM NONFERRO': 4,
        'PLASTIK': 5
    }

    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        robot(0, 0)
        global stop
        stop = 1
        return

    results = model(frame)

    # Process all boxes
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = model.names[cls]

            # Only track trash objects
            if label in TRASH_LABELS:
                if conf > rolling_best_conf:
                    rolling_best_conf = conf
                    rolling_best_label = label
                    rolling_best_box = (x1, y1, x2, y2)

    # Draw current best candidate
    if rolling_best_box:
        x1, y1, x2, y2 = rolling_best_box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f'{rolling_best_label} {rolling_best_conf:.2f}', (x1, y1 - 10), font, 1, (36, 255, 12), 2)

    # Count frames
    frame_counter += 1

    if frame_counter >= REQUIRED_FRAMES:
        if rolling_best_conf >= CONFIDENCE_THRESHOLD and rolling_best_label in TRASH_LABELS:
            center = ((x1 + x2) / 2, (y1 + y2) / 2)
            error = (center[0] - CAMERA_WIDTH // 2, center[1] - CAMERA_HEIGHT // 2)

            cv2.putText(frame, f"Axis : {error}", (5, 60), font, 1, (0, 255, 255), 2, cv2.LINE_AA)

            Trash = TRASH_LABELS[rolling_best_label]
            last_error = (0,0)
            previous_time = current_milli_time()
            counter = 2
            shutdown = 0
            sukses = 0
            robot(11,0)
            trash_type()
            cv2.putText(frame, f"{rolling_best_label}", (5, 30), font, 1, (0, 255, 255), 2, cv2.LINE_AA)

            # Reset tracker
            rolling_best_conf = 0
            rolling_best_label = None
            rolling_best_box = None
            frame_counter = 0
            conf_cn = 0
            retri = 0

        else:
            # Low confidence – trigger retry
            currentmillis = millis()
            if currentmillis - startmillis >= 1000:
                conf_cn += 1
                robot(0, 0)
                retri = 5 - conf_cn
                print("Low confidence, retrying...")
                print(f"Retry in {retri:.2f} seconds")
                if conf_cn > 5:
                    shutdown += 1
                startmillis = currentmillis

                
                
                    

            cv2.putText(frame, f"LOW Confidence :-(", (5, 30), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
           

            # Reset tracker for next round
            rolling_best_conf = 0
            rolling_best_label = None
            rolling_best_box = None
            frame_counter = 0

    if conf_cn > 5:
        shutdown_counter = 61 - shutdown
        if shutdown_counter <= 60 :
            cv2.putText(frame, f"WARNING!!!: Shutdown in {int(shutdown_counter)} sec", (50, 50), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
            if shutdown_counter == 0:
                shutdown_flag = 1
                stop = 1
            

    # Show debug window
    resized_frame = cv2.resize(frame, (screen_width, screen_height))

    cv2.imshow("Detection", resized_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        stop = 1


    

def yoloMove():
    global stop
    global counter
    global conf_cn
    global last_error
    global previous_time
    global last_conf
    global retri
    global startmillis
    global YOLO_Y_Axis
    global integral_error
    global sukses
    global tolerance

    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        robot(0, 0)
        stop = 1
        return  # Exit early

    image_center_x = CAMERA_WIDTH // 2
    image_center_y = CAMERA_HEIGHT // 2
    results = model(frame)
    best_box = None
    best_conf = 0
    closest_distance = float('inf')

    for result in results:
        boxes = result.boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = model.names[cls]

            if label == Trash_label and conf >= best_conf :
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                
                distance = ((cx - image_center_x)**2 + (cy - image_center_y)**2) ** 0.5
                if distance < closest_distance:
                    closest_distance = distance
                    best_box = box
                    best_conf = conf


    if best_box:
        x1, y1, x2, y2 = map(int, best_box.xyxy[0])
        conf = float(best_box.conf[0])
        center = ((x1 + x2) / 2, (y1 + y2) / 2)
        error = (center[0] - SETPOINT_X, center[1] - CAMERA_HEIGHT // 2)

        current_time = current_milli_time()
        dT = (current_time - previous_time) / 1000.0
        if dT == 0:
            dT = 0.001

        # PID calculation
        p_term = error
        d_term = ((error[0] - last_error[0]) / dT, (error[1] - last_error[1]) / dT)
        integral_error[0] += error[0] * dT
        integral_error[1] += error[1] * dT

        # Clamp I term
        integral_error[0] = max(min(integral_error[0], 900), -900)
        integral_error[1] = max(min(integral_error[1], 100), -100)

        # Output value
        X_Axis = int(p_term[0] * kP + integral_error[0] * kI + d_term[0] * kD)
        Y_Axis = int(p_term[1] * kP + integral_error[1] * kI + d_term[1] * kD)
        X_Axis = max(min(X_Axis, clamp_X), -clamp_X)
        Y_Axis = max(min(Y_Axis, 20), -20)
        Axis = f"{X_Axis},{Y_Axis}"

        

        # Drawing and debug
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # Compute center of rectangle
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)

        # Draw horizontal line
        cv2.line(frame, (center_x - 10, center_y), (center_x + 10, center_y), (0, 0, 255), 1)

        # Draw vertical line
        cv2.line(frame, (center_x, center_y - 10), (center_x, center_y + 10), (0, 0, 255), 1)
        
        cv2.line(frame, (SETPOINT_X, 0), (SETPOINT_X, CAMERA_HEIGHT), (255, 255, 0), 2)
        cv2.line(frame, (0, int((-60 + Y_bias) + (CAMERA_HEIGHT // 2))), (CAMERA_WIDTH, int((-60 + Y_bias) + (CAMERA_HEIGHT // 2))), (0, 0, 255), 2)
        cv2.line(frame, (0, int((-39 + Y_bias) + (CAMERA_HEIGHT // 2))), (CAMERA_WIDTH, int((-39 + Y_bias) + (CAMERA_HEIGHT // 2))), (0, 127, 255), 2)
        cv2.line(frame, (0, int((-7.5 + Y_bias) + (CAMERA_HEIGHT // 2))), (CAMERA_WIDTH, int((-7.5 + Y_bias) + (CAMERA_HEIGHT // 2))), (0, 127, 255), 2)
        cv2.line(frame, (0, int((26.5 + Y_bias) + (CAMERA_HEIGHT // 2))), (CAMERA_WIDTH, int((26.5 + Y_bias) + (CAMERA_HEIGHT // 2))), (0, 127, 255), 2)
        cv2.line(frame, (0, int((63 + Y_bias) + (CAMERA_HEIGHT // 2))), (CAMERA_WIDTH, int((63 + Y_bias) + (CAMERA_HEIGHT // 2))), (0, 127, 255), 2)
        cv2.line(frame, (0, int((119 + Y_bias) + (CAMERA_HEIGHT // 2))), (CAMERA_WIDTH, int((119 + Y_bias) + (CAMERA_HEIGHT // 2))), (0, 0, 255), 2)
        cv2.putText(frame, f'{Trash_label} {conf:.2f}', (x1, y1 - 10), font, 0.9, (36, 255, 12), 2)
        cv2.putText(frame, f"X Axis : {X_Axis}", (5, 30), font, 1, (255, 0, 0), 2)
        cv2.putText(frame, f"Y Axis : {Y_Axis}", (5, 60), font, 1, (255, 0, 0), 2)
        cv2.putText(frame, f"Axis : {Axis}", (5, 90), font, 1, (255, 0, 0), 2)
        cv2.putText(frame, f"Error : {error}", (5, 120), font, 1, (0, 255, 255), 2)
        print(f"I term: {integral_error[0]:.2f}")

        if abs(error[0]) < tolerance:
            
            
            robot(0,0)
            if sukses == 5 :
                YOLO_Y_Axis = error[1]
                previous_time = current_milli_time()
                last_error = (0,0)
                integral_error[0] = 0
                integral_error[1] = 0
                counter = 3
                cv2.putText(frame, f"Exporting Y Axis : {YOLO_Y_Axis}", (5, 150), font, 1, (255, 0, 0), 2)
                time.sleep(1)
            sukses = sukses + 1
            tolerance = tolerance - 15
            if tolerance <= 10:
                tolerance = 10
            integral_error[0] = 0
            integral_error[1] = 0
            time.sleep(0.5)

        else:
            robot(10, X_Axis)

        last_error = error
        previous_time = current_time

        # Confidence tracking logic
        if abs(conf - last_conf) > 0.1:
            last_conf = conf

    else:
        previous_time = current_milli_time()
        last_error = (0,0)
        integral_error[0] = 0
        integral_error[1] = 0
        currentmillis = millis()
        if currentmillis - startmillis >= 1000:
            conf_cn += 1
            robot(0, 0)
            retri = 5 - conf_cn
            print("Low confidence, retry in:", retri)
            startmillis = currentmillis
            if conf_cn > 5:
                last_conf = 0
                conf_cn = 0
                
                counter = 1
        cv2.putText(frame, f"LOW Confidence :-( Retry in = {retri}", (5, 30), font, 1, (0, 0, 255), 2)

    resized_frame = cv2.resize(frame, (screen_width, screen_height))

    cv2.imshow("Detection", resized_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        stop = 1



def moveArm():
    global stop
    global counter
    global conf_cn
    global previous_time
    global last_error
    global integral_error
    global YOLO_Y_Axis
    global sukses
    global tolerance
    tolerance = 50
    sukses = 0
    conf_cn = 0

    robot(0, 0)  # reset command

    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        stop = 1
        return

    # Display YOLO Y-axis info on screen
    cv2.putText(frame, f"YAxis from YOLO = {YOLO_Y_Axis}", (5, 30), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(frame, f"{Trash_label}", (5, 60), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
    cv2.line(frame, (0, int((-39 + Y_bias) + (CAMERA_HEIGHT // 2))), (CAMERA_WIDTH, int((-39 + Y_bias) + (CAMERA_HEIGHT // 2))), (0, 127, 255), 2)
    cv2.line(frame, (0, int((-7.5 + Y_bias) + (CAMERA_HEIGHT // 2))), (CAMERA_WIDTH, int((-7.5 + Y_bias) + (CAMERA_HEIGHT // 2))), (0, 127, 255), 2)
    cv2.line(frame, (0, int((26.5 + Y_bias) + (CAMERA_HEIGHT // 2))), (CAMERA_WIDTH, int((26.5 + Y_bias) + (CAMERA_HEIGHT // 2))), (0, 127, 255), 2)
    cv2.line(frame, (0, int((63 + Y_bias) + (CAMERA_HEIGHT // 2))), (CAMERA_WIDTH, int((63 + Y_bias) + (CAMERA_HEIGHT // 2))), (0, 127, 255), 2)
    cv2.line(frame, (0, int((119 + Y_bias) + (CAMERA_HEIGHT // 2))), (CAMERA_WIDTH, int((119 + Y_bias) + (CAMERA_HEIGHT // 2))), (0, 127, 255), 2)
    resized_frame = cv2.resize(frame, (screen_width, screen_height))
    cv2.imshow("Detection", resized_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        stop = 1
        return

    # === Servo movement presets ===
    servoup   = [30, 35, 45, 45, 63, 40]     # wrist
    servomid  = [85, 80, 55, 45, 23, 45]     # elbow
    servobot  = [5, 150, 300, 350, 500, 700]  # shoulder PWM



    YOLO_Y_Axis = YOLO_Y_Axis - Y_bias
    # === Determine index from YOLO_Y_Axis ===
    if YOLO_Y_Axis <= -60:
        index = 5
        counter = 1
        return
    if -60 < YOLO_Y_Axis and YOLO_Y_Axis <= -39:
        print("Arm Posisi Paling Atas")
        index = 0  # posisi paling atas
    if -39 < YOLO_Y_Axis and YOLO_Y_Axis <= -7.5:
        print("Arm Posisi Radak Atas")
        index = 1  # posisi rada atas
    if -7.5 < YOLO_Y_Axis and YOLO_Y_Axis <= 26.5:
        print("Arm Posisi Tengah")
        index = 2  # posisi tengah
    if 26.5 < YOLO_Y_Axis and YOLO_Y_Axis <= 63:
        print("Arm Posisi Radak Bawah")
        index = 3  # posisi rada bawah
    if 63 < YOLO_Y_Axis and YOLO_Y_Axis <= 119:
        print("Arm Posisi Paling Bawah")
        index = 4  # posisi paling bawah
    if 119 < YOLO_Y_Axis:
        index = 5
        counter = 1
        return
        
    # === Move to prepick position ===
    slowServo(95, 45, 1)      # elbow to prepick
    time.sleep(robotpause)
    robot(4, 900)             # shoulder to prepick   
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        stop = 1
        return
    if index == 0 :
        cv2.putText(frame, f"Posisi Paling Jauh", (5, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        resized_frame = cv2.resize(frame, (screen_width, screen_height))
        cv2.imshow("Detection", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop = 1
            return    
    if index == 1 :
        cv2.putText(frame, f"Posisi Radak Jauh", (5, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        resized_frame = cv2.resize(frame, (screen_width, screen_height))
        cv2.imshow("Detection", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop = 1
            return    
    if index == 2 :
        cv2.putText(frame, f"Posisi Tengah", (5, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        resized_frame = cv2.resize(frame, (screen_width, screen_height))
        cv2.imshow("Detection", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop = 1
            return    
    if index == 3 :
        cv2.putText(frame, f"Posisi Radak Dekat", (5, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        resized_frame = cv2.resize(frame, (screen_width, screen_height))
        cv2.imshow("Detection", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop = 1
            return    
    if index == 4 :
        cv2.putText(frame, f"Posisi Paling Dekat", (5, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        resized_frame = cv2.resize(frame, (screen_width, screen_height))
        cv2.imshow("Detection", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop = 1
            return    
    if index == 5 :
        cv2.putText(frame, f"ARM OUT OFF RANGE!!!", (5, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        resized_frame = cv2.resize(frame, (screen_width, screen_height))
        cv2.imshow("Detection", resized_frame)
        counter = 1
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop = 1
            return  
        return
    # === Move to Prepick Position ===
    time.sleep(bigservopause)
    slowServo(45, 40, 0)       # wrist to prepick
    time.sleep(robotpause)

    # === Perform pickup ===
    time.sleep(robotpause)
    slowServo(40, servoup[index], 0)      # wrist (atas)
    time.sleep(robotpause)
    slowServo(45, servomid[index], 1)     # elbow (tengah)
    time.sleep(robotpause)
    robot(4, servobot[index])             # shoulder/down (PWM)
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        stop = 1
        return
    if index == 0 :
        cv2.putText(frame, f"Posisi Paling Jauh", (5, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        resized_frame = cv2.resize(frame, (screen_width, screen_height))
        cv2.imshow("Detection", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop = 1
            return    
    if index == 1 :
        cv2.putText(frame, f"Posisi Radak Jauh", (5, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        resized_frame = cv2.resize(frame, (screen_width, screen_height))
        cv2.imshow("Detection", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop = 1
            return    
    if index == 2 :
        cv2.putText(frame, f"Posisi Tengah", (5, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        resized_frame = cv2.resize(frame, (screen_width, screen_height))
        cv2.imshow("Detection", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop = 1
            return    
    if index == 3 :
        cv2.putText(frame, f"Posisi Radak Dekat", (5, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        resized_frame = cv2.resize(frame, (screen_width, screen_height))
        cv2.imshow("Detection", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop = 1
            return    
    if index == 4 :
        cv2.putText(frame, f"Posisi Paling Dekat", (5, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        resized_frame = cv2.resize(frame, (screen_width, screen_height))
        cv2.imshow("Detection", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop = 1
            return    
    time.sleep(bigservopause)
    robot(7, 1)                            # pump ON (suction)

    # === Return to standby ===
    time.sleep(5)
    robot(4, 900)                          # half-lift shoulder
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        stop = 1
        return
    if index == 0 :
        cv2.putText(frame, f"Posisi Paling Jauh", (5, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        resized_frame = cv2.resize(frame, (screen_width, screen_height))
        cv2.imshow("Detection", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop = 1
            return    
    if index == 1 :
        cv2.putText(frame, f"Posisi Radak Jauh", (5, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        resized_frame = cv2.resize(frame, (screen_width, screen_height))
        cv2.imshow("Detection", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop = 1
            return    
    if index == 2 :
        cv2.putText(frame, f"Posisi Tengah", (5, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        resized_frame = cv2.resize(frame, (screen_width, screen_height))
        cv2.imshow("Detection", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop = 1
            return    
    if index == 3 :
        cv2.putText(frame, f"Posisi Radak Dekat", (5, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        resized_frame = cv2.resize(frame, (screen_width, screen_height))
        cv2.imshow("Detection", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop = 1
            return    
    if index == 4 :
        cv2.putText(frame, f"Posisi Paling Dekat", (5, 30), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        resized_frame = cv2.resize(frame, (screen_width, screen_height))
        cv2.imshow("Detection", resized_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop = 1
            return    
    time.sleep(bigservopause)
    slowServo(servomid[index], 95, 1)     # elbow reset
    time.sleep(robotpause)
    slowServo(servoup[index], 45, 0)       # wrist reset
    time.sleep(robotpause)
    robot(4, 2100)                         # shoulder full standby
    time.sleep(bigservopause)

    previous_time = current_milli_time()
    last_error = (0,0)
    integral_error[0] = 0
    integral_error[1] = 0
    robot(11,0)
    counter = 4
    


def cvMove():
    global conf_cn
    global stop
    global counter
    global previous_time
    global integral_error
    global last_error
    global sukses
    global tolerance
    conf_cn = 0
    
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        robot(0, 0)
        stop = 1
        return  # Exit early
    
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, COLOR_LOWER, COLOR_UPPER)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), r) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        if M["m00"] == 0:
            robot(0,0)
            return
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        error = (center[0] - SETPOINT_X, center[1] - CAMERA_HEIGHT // 2)

        current_time = current_milli_time()
        dT = (current_time - previous_time) / 1000.0
        if dT == 0:
            dT = 0.001

        # PID calculation
        p_term = error
        d_term = ((error[0] - last_error[0]) / dT, (error[1] - last_error[1]) / dT)
        integral_error[0] += error[0] * dT
        integral_error[1] += error[1] * dT

        # Clamp I term
        integral_error[0] = max(min(integral_error[0], 900), -900)
        integral_error[1] = max(min(integral_error[1], 100), -100)

        # Output value
        X_Axis = int(p_term[0] * kP + integral_error[0] * kI + d_term[0] * kD)
        Y_Axis = int(p_term[1] * kP + integral_error[1] * kI + d_term[1] * kD)
        X_Axis = max(min(X_Axis, clamp_X), -clamp_X)
        Y_Axis = max(min(Y_Axis, 20), -20)
        Axis = f"{X_Axis},{Y_Axis}"
        cv2.line(frame, (SETPOINT_X, 0), (SETPOINT_X, CAMERA_HEIGHT), (255, 255, 0), 2)
        cv2.putText(frame, f"X Axis : {X_Axis}", (5, 30), font, 1, (255, 0, 0), 2)
        cv2.putText(frame, f"Y Axis : {Y_Axis}", (5, 60), font, 1, (255, 0, 0), 2)
        cv2.putText(frame, f"Axis : {Axis}", (5, 90), font, 1, (255, 0, 0), 2)
        cv2.putText(frame, f"Error : {error}", (5, 120), font, 1, (0, 255, 255), 2)
        cv2.putText(frame, f"{Trash_label}", (5, 150), font, 1, (255, 0, 0), 2, cv2.LINE_AA)

        if abs(error[0]) < tolerance:
            if sukses == 5 :
                robot(0,0)
                last_error = (0,0)
                integral_error[0] = 0
                integral_error[1] = 0
                counter = 5
                previous_time = current_milli_time()
                time.sleep(1)
            integral_error[0] = 0
            integral_error[1] = 0                
            sukses = sukses + 1
            tolerance = tolerance - 15
            if tolerance <= 10:
                tolerance = 10
            time.sleep(0.5)

        if r > 10:
            time.sleep(0.065)
            robot(10, X_Axis)
            cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 255), 2)
        else:
            previous_time = current_milli_time()
            last_error = (0,0)
            integral_error[0] = 0
            integral_error[1] = 0
        
        last_error = error
        previous_time = current_time

    resized_frame = cv2.resize(frame, (screen_width, screen_height))

    cv2.imshow("Detection", resized_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        stop = 1

    

def putToTrash():
    global stop
    global counter
    global conf_cn
    global previous_time
    global last_error
    global integral_error
    conf_cn = 0

    robot(0, 0)  # reset command

    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        stop = 1
        return

    # Display YOLO Y-axis info on screen
    
    cv2.putText(frame, f"{Trash_label}", (5, 30), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
    resized_frame = cv2.resize(frame, (screen_width, screen_height))

    cv2.imshow("Detection", resized_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        stop = 1
        return
    # === Move to pretrash position ===
    print("PreTarsh Position")
    robot(4, 1000)             # shoulder to pretrash
    time.sleep(bigservopause)
    slowServo(45, 80, 0)       # wrist to pretrash
    time.sleep(robotpause)
    slowServo(95, 90, 1)      # elbow to pretrash
    time.sleep(robotpause)
    # === Move to Trash ===
    print("Put to Trash")
    robot(4, 5)             # shoulder to trash
    time.sleep(bigservopause)
    robot(7,2)              # Pump Blow Trash
    time.sleep(10)
    robot(7,0)              #Turn Off Pump
    time.sleep(2)
    # === Move to StandBY ===
    print("Back To StandBy")
    robot(4, 1000)             # shoulder to pre standby
    time.sleep(bigservopause)
    slowServo(80, 45, 0)        # wrist to standby
    time.sleep(robotpause)
    slowServo(90,95,1)        # elbow to standby
    time.sleep(robotpause)
    robot(4,2100)              # shoulder to standby
    time.sleep(bigservopause)
    previous_time = current_milli_time()
    last_error = (0,0)
    integral_error[0] = 0
    integral_error[1] = 0
    
    counter = 1
    
    


def setup():
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        global stop
        stop = 1
    global counter
    global data
    global last_error
    global previous_time
    global last_conf
    global maju
    last_error = (0,0)
    previous_time = current_milli_time()
    last_conf = 0
    print("Test")
    slowServo(93,95,1)        # elbow to standby
    time.sleep(robotpause)
    slowServo(20, 45, 0)        # wrist to standby
    time.sleep(robotpause)
    # Set up fullscreen window
    cv2.namedWindow("Detection", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Detection", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    robot(7,1)
    time.sleep(5)
    robot(7,2)
    time.sleep(5)
    robot(7,0)
    resized_frame = cv2.resize(frame, (screen_width, screen_height))
    cv2.imshow("Detection", resized_frame)
    cv2.waitKey(1)
    time.sleep(0.1)
    if maju == 0:
        robot(9,0)
        time.sleep(0.1)
        data = ser.read(2)
        print("Start")
        counter = 1
        time.sleep(0.1)
        robot(0,0)
        maju = 1

    

def loop():

    if counter == 1:
        
        yoloScan()
    if counter == 2:
        
        yoloMove()
    if counter == 3:

        moveArm()
    if counter == 4:

        cvMove()
    if counter == 5:

        putToTrash()
    


setup()
while True:
    
    loop()
    if stop == 1 :
        robot(0,0)
        slowServo(45, 20, 0)        # wrist to standby
        time.sleep(robotpause)
        slowServo(95,2,1)        # elbow to standby
        time.sleep(robotpause)
        robot(4,2100)              # shoulder to standby
        time.sleep(bigservopause)            
        
        resetArduino()
        cap.release()
        cv2.destroyAllWindows()
        if shutdown_flag == 1:
            if platform.system() == "Windows":
                os.system("shutdown /s /t 1")
            elif platform.system() == "Linux":
                os.system("sudo shutdown now")

        break