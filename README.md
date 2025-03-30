Key Functionalities:
Object Detection using TensorFlow Lite (TFLite) Model:

Uses MobileNet SSD v2 COCO model for object detection.

Detects objects from a predefined list (arr_valid_objects).

Uses OpenCV to capture video and track detected objects.

Ultrasonic Sensor for Obstacle Detection:

Measures the distance of obstacles using an HC-SR04 sensor.

Prevents collision by stopping the robot if an obstacle is too close.

Uses SAFE_DISTANCE = 30cm and CRITICAL_DISTANCE = 15cm as thresholds.

Flask Web Interface for Video Streaming:

Runs a Flask server (app = Flask(__name__)).

Streams live video feed using /video_feed endpoint.

Motor Control using PWM on GPIO Pins:

Uses GPIO pins 20 and 21 for motor speed control.

Adjusts speed dynamically based on distance to obstacles.

Object Tracking Algorithm:

Calculates x_deviation and y_deviation from the center of the frame.

Moves the robot left/right if deviation is in the x-axis.

Moves the robot forward/backward if deviation is in the y-axis.

Uses a separate thread (Thread(target=move_robot)) for movement.

Speed Adjustment Logic:

If the robot is too close to an obstacle, it stops.

If within a safe range, speed is reduced dynamically.

Possible Issues & Improvements
Timeout Handling in Ultrasonic Sensor:

The function measure_distance() includes timeouts, but excessive sensor noise can still cause errors.

Consider adding an average filter (like a moving average) to smooth out sensor readings.

CPU Performance Optimization:

cap = cv2.VideoCapture(0) runs on the main thread, which might block performance.

Solution: Move video capture and processing to a separate thread to prevent lag.

Handling Cases When No Object is Found:

The function track_object(objs, labels) stops the robot when no object is detected.

But it does not attempt to search for the object again.

Solution: Implement a scanning behavior (slow rotation) to re-acquire objects.

Speed Control Could Be Smoother:

The speed changes suddenly in adjust_speed_by_distance(distance).

Solution: Implement gradual acceleration and deceleration instead of direct speed jumps.
