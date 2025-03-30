import common as cm
import cv2
import numpy as np
from PIL import Image
import time
from threading import Thread

import sys
sys.path.append('/home/admin/robotics-level-4/earthrover')
import util as ut
ut.init_gpio()

cap = cv2.VideoCapture(0)
threshold = 0.2
top_k = 5  # number of objects to be shown as detected

model_dir = '/home/admin/robotics-level-4/all_models'
model = 'mobilenet_ssd_v2_coco_quant_postprocess.tflite'
model_edgetpu = 'mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite'
lbl = 'coco_labels.txt'

tolerance = 0.1
x_deviation = 0
y_deviation = 0
arr_track_data = [0, 0, 0, 0, 0, 0]

arr_valid_objects = ['apple', 'sports ball', 'frisbee', 'orange', 'mouse', 'vase', 'banana']

# --------- Ultrasonic Sensor Setup -----------------
import RPi.GPIO as GPIO
import time

# GPIO pins for ultrasonic sensor
TRIG_FRONT = 23  # GPIO pin for front ultrasonic trigger
ECHO_FRONT = 24  # GPIO pin for front ultrasonic echo

# Distance thresholds (in cm)
SAFE_DISTANCE = 30
CRITICAL_DISTANCE = 15
MAX_DISTANCE = 200  # Maximum distance measurement (cm)

# Setup GPIO for ultrasonic sensor
def setup_ultrasonic():
    GPIO.setup(TRIG_FRONT, GPIO.OUT)
    GPIO.setup(ECHO_FRONT, GPIO.IN)
    GPIO.output(TRIG_FRONT, False)
    time.sleep(0.2)  # Let sensor settle

# Function to measure distance with ultrasonic sensor
def measure_distance():
    # Send trigger pulse
    GPIO.output(TRIG_FRONT, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_FRONT, False)
    
    start_time = time.time()
    stop_time = time.time()
    
    # Get pulse start time
    timeout_start = time.time()
    while GPIO.input(ECHO_FRONT) == 0:
        start_time = time.time()
        # Add timeout to prevent hanging if sensor fails
        if time.time() - timeout_start > 0.1:
            return MAX_DISTANCE
    
    # Get pulse end time
    timeout_start = time.time()
    while GPIO.input(ECHO_FRONT) == 1:
        stop_time = time.time()
        # Add timeout to prevent hanging if sensor fails
        if time.time() - timeout_start > 0.1:
            return MAX_DISTANCE
    
    # Calculate distance in cm
    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300) / 2  # Speed of sound = 343 m/s
    
    return min(distance, MAX_DISTANCE)  # Cap at max detection distance

# ---------Flask----------------------------------------
from flask import Flask, Response
from flask import render_template

app = Flask(__name__)

@app.route('/')
def index():
    return render_template("index.html")

@app.route('/video_feed')
def video_feed():
    return Response(main(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
                    
# -----------------------------------------------------------

# -----initialise motor speed-----------------------------------

GPIO.setmode(GPIO.BCM)  # choose BCM numbering scheme  
      
GPIO.setup(20, GPIO.OUT)  # set GPIO 20 as output pin
GPIO.setup(21, GPIO.OUT)  # set GPIO 21 as output pin
      
pin20 = GPIO.PWM(20, 100)    # create object pin20 for PWM on port 20 at 100 Hertz  
pin21 = GPIO.PWM(21, 100)    # create object pin21 for PWM on port 21 at 100 Hertz  

# set speed to maximum value
val = 100
pin20.start(val)              # start pin20 on 0 percent duty cycle (off)  
pin21.start(val)              # start pin21 on 0 percent duty cycle (off)  
    
print("speed set to: ", val)
# ---------------------------------------------------------------

# Debug mode - print expected vs actual movements    
DEBUG_DIRECTION = True

def track_object(objs, labels):
    global x_deviation, y_deviation, tolerance, arr_track_data
    
    if len(objs) == 0:
        print("no objects to track")
        ut.stop()
        ut.red_light("OFF")
        arr_track_data = [0, 0, 0, 0, 0, 0]
        return

    k = 0
    flag = 0
    for obj in objs:
        lbl = labels.get(obj.id, obj.id)
        k = arr_valid_objects.count(lbl)
        if k > 0:
            x_min, y_min, x_max, y_max = list(obj.bbox)
            flag = 1
            break
        
    if flag == 0:
        print("selected object not present")
        return
        
    x_diff = x_max - x_min
    y_diff = y_max - y_min
    
    # Calculate center of the object
    obj_x_center = x_min + (x_diff / 2)
    obj_x_center = round(obj_x_center, 3)
    
    obj_y_center = y_min + (y_diff / 2)
    obj_y_center = round(obj_y_center, 3)
    
    # Calculate deviations from center - IMPORTANT: These determine the movement direction
    # The key is to understand the coordinate system:
    # x_deviation positive means object is to the left of center
    # x_deviation negative means object is to the right of center
    # y_deviation positive means object is above the center (closer to top of frame)
    # y_deviation negative means object is below the center (closer to bottom of frame)
    
    x_deviation = round(0.5 - obj_x_center, 3)
    y_deviation = round(0.5 - obj_y_center, 3)
    
    if DEBUG_DIRECTION:
        print("Object center: [{:.3f}, {:.3f}]".format(obj_x_center, obj_y_center))
        print("Deviation: x={:.3f}, y={:.3f}".format(x_deviation, y_deviation))
        
        # Calculate expected movement based on deviation
        expected_x_movement = "LEFT" if x_deviation > 0 else "RIGHT" if x_deviation < 0 else "CENTER"
        expected_y_movement = "FORWARD" if y_deviation < 0 else "BACKWARD" if y_deviation > 0 else "CENTER"
        
        print("Expected movement: Horizontal={}, Vertical={}".format(expected_x_movement, expected_y_movement))
    
    thread = Thread(target=move_robot)
    thread.start()
    
    arr_track_data[0] = obj_x_center
    arr_track_data[1] = obj_y_center
    arr_track_data[2] = x_deviation
    arr_track_data[3] = y_deviation

# Function to adjust speed based on distance to obstacle
def adjust_speed_by_distance(distance):
    # Map distance to speed (linear relationship)
    if distance <= CRITICAL_DISTANCE:
        return 0  # Stop completely
    elif distance <= SAFE_DISTANCE:
        # Gradually reduce speed as we get closer to obstacle
        speed_factor = (distance - CRITICAL_DISTANCE) / (SAFE_DISTANCE - CRITICAL_DISTANCE)
        return max(30, int(speed_factor * 100))  # Minimum speed of 30%
    else:
        return 100  # Full speed when safe

# This function is executed within a thread
def move_robot():
    global x_deviation, y_deviation, tolerance, arr_track_data
    
    print("moving robot .............!!!!!!!!!!!!!!")
    
    # Get the current distance from ultrasonic sensor
    distance = measure_distance()
    print(f"Distance to obstacle: {distance} cm")
    
    # Set the default command and delay
    cmd = "Stop"
    delay1 = 0
    
    # Check if we're already aligned (within tolerance)
    if abs(x_deviation) < tolerance and abs(y_deviation) < tolerance:
        ut.stop()
        ut.red_light("ON")
    else:
        ut.red_light("OFF")
        
        # FIX: Corrected movement logic based on deviations
        # Decide direction based on larger deviation
        if abs(x_deviation) > abs(y_deviation):
            # Left-right movement (no need to check distance for side movement)
            if x_deviation > tolerance:  # Object is to the left of center
                cmd = "Move Right"  # FIXED: Move right to center the object
                delay1 = get_delay(x_deviation, 'r')
                
                ut.right()  # FIXED: Changed from left() to right()
                time.sleep(delay1)
                ut.stop()
                
            elif x_deviation < -1 * tolerance:  # Object is to the right of center
                cmd = "Move Left"  # FIXED: Move left to center the object
                delay1 = get_delay(x_deviation, 'l')
                
                ut.left()  # FIXED: Changed from right() to left()
                time.sleep(delay1)
                ut.stop()
        else:
            # Forward-backward movement
            if y_deviation < -1 * tolerance:  # Object is below center (closer to bottom)
                # Moving forward - check distance first
                if distance > CRITICAL_DISTANCE:
                    cmd = "Move Forward"  # FIXED: Move forward when object is below center
                    # Adjust speed based on distance
                    speed_percent = adjust_speed_by_distance(distance)
                    if speed_percent > 0:
                        # Set adjusted speed for motors
                        pin20.ChangeDutyCycle(speed_percent)
                        pin21.ChangeDutyCycle(speed_percent)
                        
                        delay1 = get_delay(y_deviation, 'f')
                        
                        ut.forward()  # FIXED: Changed from back() to forward()
                        time.sleep(delay1)
                        ut.stop()
                        
                        # Reset speed to maximum
                        pin20.ChangeDutyCycle(100)
                        pin21.ChangeDutyCycle(100)
                    else:
                        cmd = "Too Close - Cannot Move Forward"
                else:
                    cmd = "Too Close - Cannot Move Forward"
                
            elif y_deviation > tolerance:  # Object is above center (closer to top)
                cmd = "Move Backward"  # FIXED: Move backward when object is above center
                delay1 = get_delay(y_deviation, 'b')
                
                ut.back()  # FIXED: Changed from forward() to back()
                time.sleep(delay1)
                ut.stop()
    
    arr_track_data[4] = cmd
    arr_track_data[5] = delay1

# Based on the deviation of the object from the center of the frame
def get_delay(deviation, direction):
    deviation = abs(deviation)
    if direction == 'f' or direction == 'b':
        if deviation >= 0.3:
            d = 0.1
        elif deviation >= 0.2 and deviation < 0.30:
            d = 0.075
        elif deviation >= 0.15 and deviation < 0.2:
            d = 0.045
        else:
            d = 0.035
    else:
        if deviation >= 0.4:
            d = 0.080
        elif deviation >= 0.35 and deviation < 0.40:
            d = 0.070
        elif deviation >= 0.30 and deviation < 0.35:
            d = 0.060
        elif deviation >= 0.25 and deviation < 0.30:
            d = 0.050
        elif deviation >= 0.20 and deviation < 0.25:
            d = 0.040
        else:
            d = 0.030
    
    return d

def main():
    # Setup ultrasonic sensor
    setup_ultrasonic()
    
    from util import edgetpu
    
    if edgetpu == 1:
        mdl = model_edgetpu
    else:
        mdl = model
        
    interpreter, labels = cm.load_model(model_dir, mdl, lbl, edgetpu)
    
    fps = 1
    arr_dur = [0, 0, 0]
    
    while True:
        start_time = time.time()
        
        # ----------------Capture Camera Frame-----------------
        start_t0 = time.time()
        ret, frame = cap.read()
        if not ret:
            break
        
        cv2_im = frame
        cv2_im = cv2.flip(cv2_im, 0)
        cv2_im = cv2.flip(cv2_im, 1)

        cv2_im_rgb = cv2.cvtColor(cv2_im, cv2.COLOR_BGR2RGB)
        pil_im = Image.fromarray(cv2_im_rgb)
       
        arr_dur[0] = time.time() - start_t0
        # ----------------------------------------------------
       
        # -------------------Inference---------------------------------
        start_t1 = time.time()
        cm.set_input(interpreter, pil_im)
        interpreter.invoke()
        objs = cm.get_output(interpreter, score_threshold=threshold, top_k=top_k)
        
        arr_dur[1] = time.time() - start_t1
        # ----------------------------------------------------
       
        # -----------------other------------------------------------
        start_t2 = time.time()
        
        # Get distance reading before tracking
        distance = measure_distance()
        
        # Track the object with the distance information
        track_object(objs, labels)
       
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        # Pass distance information to the overlay drawing function
        cv2_im = draw_overlays(cv2_im, objs, labels, arr_dur, arr_track_data, distance)
        
        ret, jpeg = cv2.imencode('.jpg', cv2_im)
        pic = jpeg.tobytes()
        
        # Flask streaming
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + pic + b'\r\n\r\n')
       
        arr_dur[2] = time.time() - start_t2
        
        fps = round(1.0 / (time.time() - start_time), 1)
        print("FPS: ", fps, "*")

    cap.release()
    cv2.destroyAllWindows()

def draw_overlays(cv2_im, objs, labels, arr_dur, arr_track_data, distance=None):
    height, width, channels = cv2_im.shape
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    global tolerance
    
    # Draw black rectangle on top
    cv2_im = cv2.rectangle(cv2_im, (0, 0), (width, 24), (0, 0, 0), -1)
    
     
    # Write processing durations
    cam = round(arr_dur[0] * 1000, 0)
    inference = round(arr_dur[1] * 1000, 0)
    other = round(arr_dur[2] * 1000, 0)
    text_dur = 'Camera: {}ms   Inference: {}ms   other: {}ms'.format(cam, inference, other)
    cv2_im = cv2.putText(cv2_im, text_dur, (int(width/4) - 30, 16), font, 0.4, (255, 255, 255), 1)
    
    # Write FPS 
    total_duration = cam + inference + other
    fps = round(1000 / total_duration, 1)
    text1 = 'FPS: {}'.format(fps)
    cv2_im = cv2.putText(cv2_im, text1, (10, 20), font, 0.7, (150, 150, 255), 2)
   
    
    # Draw black rectangle at bottom
    cv2_im = cv2.rectangle(cv2_im, (0, height - 48), (width, height), (0, 0, 0), -1)
    
    # Write deviations and tolerance
    str_tol = 'Tol : {}'.format(tolerance)
    cv2_im = cv2.putText(cv2_im, str_tol, (10, height - 32), font, 0.55, (150, 150, 255), 2)
   
    x_dev = arr_track_data[2]
    str_x = 'X: {}'.format(x_dev)
    if abs(x_dev) < tolerance:
        color_x = (0, 255, 0)
    else:
        color_x = (0, 0, 255)
    cv2_im = cv2.putText(cv2_im, str_x, (110, height - 32), font, 0.55, color_x, 2)
    
    y_dev = arr_track_data[3]
    str_y = 'Y: {}'.format(y_dev)
    if abs(y_dev) < tolerance:
        color_y = (0, 255, 0)
    else:
        color_y = (0, 0, 255)
    cv2_im = cv2.putText(cv2_im, str_y, (220, height - 32), font, 0.55, color_y, 2)
    
    # Add movement direction visualization
    if DEBUG_DIRECTION and (abs(x_dev) > tolerance or abs(y_dev) > tolerance):
        # Add a visual indicator for expected movement direction
        if abs(x_dev) > abs(y_dev):
            # Horizontal movement is primary
            direction = "→" if x_dev < 0 else "←" if x_dev > 0 else ""
        else:
            # Vertical movement is primary
            direction = "↑" if y_dev < 0 else "↓" if y_dev > 0 else ""
            
        # Add the direction indicator to the display
        cv2_im = cv2.putText(cv2_im, direction, (int(width/2) - 20, 70), 
                             font, 1.5, (0, 255, 255), 3)
    
    # Add distance information from ultrasonic sensor
    if distance is not None:
        # Color-code based on distance (green if safe, yellow if approaching critical, red if too close)
        if distance > SAFE_DISTANCE:
            dist_color = (0, 255, 0)  # Green
        elif distance > CRITICAL_DISTANCE:
            dist_color = (0, 255, 255)  # Yellow
        else:
            dist_color = (0, 0, 255)  # Red
            
        str_dist = 'Dist: {} cm'.format(round(distance, 1))
        cv2_im = cv2.putText(cv2_im, str_dist, (width - 200, height - 32), font, 0.55, dist_color, 2)
    
    # Write direction, speed, tracking status
    cmd = arr_track_data[4]
    cv2_im = cv2.putText(cv2_im, str(cmd), (int(width/2) + 10, height - 8), font, 0.68, (0, 255, 255), 2)
    
    delay1 = arr_track_data[5]
    str_sp = 'Speed: {}%'.format(round(delay1/(0.1) * 100, 1))
    cv2_im = cv2.putText(cv2_im, str_sp, (int(width/2) + 185, height - 8), font, 0.55, (150, 150, 255), 2)
    
    if cmd == 0:
        str1 = "No object"
    elif cmd == 'Stop':
        str1 = 'Acquired'
    elif cmd == 'Too Close - Cannot Move Forward':
        str1 = 'Obstacle Detected'
    else:
        str1 = 'Tracking'
    cv2_im = cv2.putText(cv2_im, str1, (width - 140, 18), font, 0.7, (0, 255, 255), 2)
    
    # Draw center cross lines
    cv2_im = cv2.rectangle(cv2_im, (0, int(height/2) - 1), (width, int(height/2) + 1), (255, 0, 0), -1)
    cv2_im = cv2.rectangle(cv2_im, (int(width/2) - 1, 0), (int(width/2) + 1, height), (255, 0, 0), -1)
    
    # Draw the center red dot on the object
    cv2_im = cv2.circle(cv2_im, (int(arr_track_data[0] * width), int(arr_track_data[1] * height)), 7, (0, 0, 255), -1)

    # Draw the tolerance box
    cv2_im = cv2.rectangle(cv2_im, 
                          (int(width/2 - tolerance * width), int(height/2 - tolerance * height)), 
                          (int(width/2 + tolerance * width), int(height/2 + tolerance * height)), 
                          (0, 255, 0), 2)
    
    # Draw bounding boxes
    for obj in objs:
        x0, y0, x1, y1 = list(obj.bbox)
        x0, y0, x1, y1 = int(x0 * width), int(y0 * height), int(x1 * width), int(y1 * height)
        percent = int(100 * obj.score)
        
        box_color, text_color, thickness = (0, 150, 255), (0, 255, 0), 2
        cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), box_color, thickness)
        
    return cv2_im

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=2204, threaded=True)  # Run FLASK
        main()
    finally:
        # Clean up GPIO
        GPIO.cleanup()