# Imports

import cv2
import numpy as np
from filterpy.kalman import KalmanFilter
import paho.mqtt.client as mqtt
import zlib
import base64
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from queue import Queue
from ament_index_python.packages import get_package_share_path
from pathlib import Path



# MQTT configuration

broker_address = "mqtt-dashboard.com"
port = 1883
topic = "testtopic/webcam_mqttcast"

client = mqtt.Client()
client.connect(broker_address, port, keepalive = 60)

exit_flag = False

# Defs

def initialize_mqtt():
    client = mqtt.Client()
    client.on_message = on_message
    client.connect(broker_address, port, keepalive=60)
    client.subscribe(topic)
    return client

def initialize_camera(device_index):
    cap = cv2.VideoCapture(device_index)
    return cap

def load_aruco_detector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_params.cornerRefinementMethod = 1
    aruco_params.useAruco3Detection = True
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    return detector

def detect_aruco_markers(frame, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray)
    return frame, markerCorners, markerIds

def process_aruco_markers(markerIds, markerCorners):
    mark1_points, mark2_points, mark3_points, mark4_points = None, None, None, None
    mark_weight = None
    weight = None
    height = None

    for i in range(len(markerIds)):
        if markerIds[i] == 1:
            mark1_points = markerCorners[i][0]
        elif markerIds[i] == 2:
            mark2_points = markerCorners[i][0]
        elif markerIds[i] == 3:
            mark3_points = markerCorners[i][0]
        elif markerIds[i] == 4:
            mark4_points = markerCorners[i][0]
        else:
            mark_weight = markerCorners[i][0]
            weight = math.ceil(markerIds[i]/10.0) * 10
            # print(weight)
            height = int(markerIds[i]) % 10
            # print (height)
            # if mark_weight is not None:
            #     print("Weight: " + str(weight))
         
    return mark1_points, mark2_points, mark3_points, mark4_points, mark_weight, weight, height

def find_angle(distance1, distance2, distance_between_points):
    # Convert distances to numpy arrays
    a = np.array(distance1)
    b = np.array(distance2)
    c = np.array(distance_between_points)

    # Use the law of cosines to find the angle in radians
    alpha_rad = np.arccos((a**2 + b**2 - c**2) / (2 * a * b))

    # Convert the angle to degrees
    alpha_deg = np.degrees(alpha_rad)

    return alpha_deg

def find_sides(hypotenuse, angle_alpha):
    # Convert angle to radians
    alpha_rad = np.radians(angle_alpha)

    # Calculate sides using trigonometric functions
    side_adjacent  = hypotenuse * np.cos(alpha_rad)
    side_opposite  = hypotenuse * np.sin(alpha_rad)

    return side_opposite, side_adjacent 

def estimate_pose(markerCorners, marker_ids):
    # Get the package share path for 'jekko'
    package_path = get_package_share_path("jekko")
    mean_matrix = Path(package_path) / 'camera_settings/mean_camera_matrix.txt'
    mean_coefficients = Path(package_path) / 'camera_settings/mean_distortion_coefficients.txt'

    marker_size = 50  # mm
    W_list = []
    object_points_list = []
    distance1w_real = None
    coordinate_relative = None

    for marker_id in marker_ids:
        if marker_id == 1:
            object_points = np.array([[-102.5, 294, 0], [-102.5 + marker_size, 294, 0],
                                      [-102.5 + marker_size, 294 - marker_size, 0], [-102.5, 294 - marker_size, 0]],
                                     dtype=np.float32)
        elif marker_id == 2:
            object_points = np.array([[-102.5, 0, 0], [-102.5 + marker_size, 0, 0],
                                      [-102.5 + marker_size, -marker_size, 0], [-102.5, -marker_size, 0]],
                                     dtype=np.float32)
        elif marker_id == 3:
            object_points = np.array([[102.5, 0, 0], [102.5 + marker_size, 0, 0],
                                      [102.5 + marker_size, -marker_size, 0], [102.5, -marker_size, 0]],
                                     dtype=np.float32)
        elif marker_id == 4:
            object_points = np.array([[102.5, 294, 0], [102.5 + marker_size, 294, 0],
                                      [102.5 + marker_size, 294 - marker_size, 0], [102.5, 294 - marker_size, 0]],
                                     dtype=np.float32)
        else:
            object_points = np.array([[0, 0, 0], [marker_size, 0, 0],
                                      [marker_size, marker_size, 0], [0, marker_size, 0]],
                                     dtype=np.float32)
        
        mark1_points, mark2_points, mark3_points, mark4_points, mark_weight, weight, height = process_aruco_markers(marker_ids, markerCorners)

        if 1 <= marker_id <= 4:
            marker_corners = markerCorners[0][0]

            retval, rvec, tvec = cv2.solvePnP(object_points, marker_corners, mean_matrix, mean_coefficients)
            R, _ = cv2.Rodrigues(rvec)
            T = np.concatenate((R, tvec), axis=1)
            W = np.vstack([T, [0, 0, 0, 1]])

            W_list.append(W)
            object_points_list.append(object_points)

        else:
            marker_corners = markerCorners[0][0]

            retval, rvec, tvec = cv2.solvePnP(object_points, marker_corners, mean_matrix, mean_coefficients)
            R, _ = cv2.Rodrigues(rvec)
            T = np.concatenate((R, tvec), axis=1)
            W = np.vstack([T, [0, 0, 0, 1]])

            W_list.append(W)
            object_points_list.append(object_points)
            center_mark1, center_mark2, center_mark3, center_mark4 = None, None, None, None
            xw, yw, zw = 0, 0, 0

            if mark2_points is not None and mark3_points is not None and mark_weight is not None:

                center_mark2 = tuple(map(float, mark2_points[0]))
                center_mark3 = tuple(map(float, mark3_points[0]))
                #print(center_mark3, center_mark2)
                center_weight = tuple(map(float, mark_weight[0]))

                distance2_3 = np.linalg.norm(np.array(center_mark2) - np.array(center_mark3))
                distance2_3_real = 205
                
                distance2_w = np.linalg.norm(np.array(center_mark2) - np.array(center_weight))
                distance2w_real = (distance2_w * distance2_3_real) / distance2_3
                #print(distance2w_real)

                distance3_w = np.linalg.norm(np.array(center_mark3) - np.array(center_weight))
                distance3w_real = (distance3_w * distance2_3_real) / distance2_3

                alpha = find_angle(distance2_3_real, distance2w_real, distance3w_real)
                #print(alpha)
                
                yw, xw = find_sides(distance2w_real, 90-alpha)
                zw = height
                coordinate_relative = xw, yw, zw
            elif mark1_points is not None and mark4_points is not None and mark_weight is not None:
                center_mark1 = tuple(map(float, mark1_points[0]))
                center_mark4 = tuple(map(float, mark4_points[0]))

                center_weight = tuple(map(float, mark_weight[0]))

                distance1_4 = np.linalg.norm(np.array(center_mark1) - np.array(center_mark4))
                distance1_4_real = 205
                
                distance1_w = np.linalg.norm(np.array(center_mark1) - np.array(center_weight))
                distance1w_real = (distance1_w * distance1_4_real) / distance1_4
                # print(distance1w_real)

                distance4_w = np.linalg.norm(np.array(center_mark4) - np.array(center_weight))
                distance4w_real = (distance4_w * distance1_4_real) / distance1_4

                alpha = find_angle(distance1_4_real, distance1w_real, distance4w_real)

                yw, xw = find_sides(distance1w_real, 90-alpha)
                zw = height
        
                coordinate_relative = xw - 102.5, 294 - yw, zw

            coordinate_relative = xw - 102.5, yw, zw
         

    return W_list, object_points_list, marker_ids, distance1w_real, coordinate_relative

def print_coordinates(W_list, object_points_list, marker_ids):
    for W, object_points, marker_id in zip(W_list, object_points_list, marker_ids):
        # Convert object_points to homogeneous coordinates
        object_points_homogeneous = np.hstack((object_points, np.ones((object_points.shape[0], 1))))

        # Transform object_points to real-world coordinates using W
        all_corners_real = np.dot(W, object_points_homogeneous.T).T[:, :3]


def create_bounding_quadrilateral(mark1_points, mark2_points, mark3_points, mark4_points, frame):
    if mark1_points is not None and mark2_points is not None and mark3_points is not None and mark4_points is not None:
        all_points = np.concatenate([mark1_points, mark2_points, mark3_points, mark4_points])
        hull = cv2.convexHull(all_points)
        hull_int32 = np.int32(hull)
        frame = cv2.polylines(frame, [hull_int32], isClosed=True, color=(0, 255, 0), thickness=3)
    return frame

def extract_translation(matrix):
    if matrix.shape != (4, 4):
        raise ValueError("Input matrix must be a 4x4 matrix")
     
    translation_vector = matrix[:3, 3]
    return tuple(translation_vector)

def draw_marker_points(frame, mark1_points, mark2_points, mark3_points, mark4_points, mark_weight, distance_real):
    def draw_point(frame, points, color, label):
        if points is not None and len(points) > 0:
            for i, point in enumerate(points):
                center = tuple(map(int, point))
                frame = cv2.circle(frame, center, 5, color, -1)
                cv2.putText(frame, f"{label}{i + 1} ({center[0]}, {center[1]})", center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    draw_point(frame, mark1_points, (0, 0, 255), "1")   # Points for marker 1
    draw_point(frame, mark2_points, (0, 255, 0), "2")   # Points for marker 2
    draw_point(frame, mark3_points, (255, 0, 0), "3")   # Points for marker 3
    draw_point(frame, mark4_points, (255, 255, 0), "4")  # Points for marker 4
    draw_point(frame, mark_weight, (255, 0, 255), "W")  # Points for marker Weight

    if mark1_points is not None and mark_weight is not None and distance_real is not None:
        center_mark11 = tuple(map(int, mark1_points[0]))
        center_mark12 = tuple(map(int, mark1_points[1]))
        center_mark14 = tuple(map(int, mark1_points[3]))

        center_mark1z = center_mark11[0], center_mark11[1], 50.0

        frame = cv2.line(frame, center_mark11, center_mark12, (255, 0, 0), 2)  # x
        frame = cv2.line(frame, center_mark11, center_mark14, (0, 255, 0), 2)  # y

        center_weight = tuple(map(int, mark_weight[0]))
        frame = cv2.line(frame, center_mark11, center_weight, (0, 255, 255), 2)

        # Calculate and display the distance
        distance = np.linalg.norm(np.array(center_mark11) - np.array(center_weight))
        cv2.putText(frame, f"Distance: {distance:.2f} pixels, {distance_real} mm", (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    return frame


def initialize_kalman_filter(dim_x, dim_z):
    kf = KalmanFilter(dim_x=dim_x, dim_z=dim_z)
    kf.x = np.zeros(dim_x)  # Initial state
    kf.F = np.eye(dim_x)  # State transition matrix
    kf.H = np.eye(dim_x)[:dim_z]  # Measurement matrix
    kf.P *= 1e2  # Initial covariance matrix
    kf.R = np.eye(dim_z) * 1e-1  # Measurement noise covariance
    return kf

def update_kalman_filter(kf, measurement):
    kf.predict()
    kf.update(measurement)
    return kf.x

moving_average_window = 200  # You can adjust the window size as needed
moving_average_buffer = {str(i): [] for i in range(1, 5)}  # For markers 1 to 4
moving_average_buffer['Weight'] = []  # For the Weight marker

def calculate_moving_average(marker_id, measurement):
    buffer = moving_average_buffer[str(marker_id)]
    buffer.append(measurement)
    if len(buffer) > moving_average_window:
        buffer.pop(0)  # Remove the oldest measurement
    return np.mean(buffer, axis=0)

def create_publisher(node, topic, msg_ros):
    publisher = node.create_publisher(String, topic, 10)
    timer_period = 0.5  # seconds
    timer = node.create_timer(timer_period, lambda: publisher_callback(node, publisher, msg_ros))
    return publisher, timer


def publisher_callback(node, publisher, msg_ros):
    try:
        # Get the message from the Queue
        msg_ros = msg_queue.get_nowait()

        msg = String()
        msg.data = msg_ros
        publisher.publish(msg)
        node.get_logger().info('Publishing: "%s"' % msg.data)

    except Exception as e:
        pass  # Handle the exception as needed


args=None
rclpy.init(args=args)
msg_ros = ""
# Create a Queue for communication between threads
msg_queue = Queue()

def on_message(client, userdata, msg):
    global exit_flag
    
    try:
        # Decode and decompress the received base64 string
        base64_data = msg.payload.decode('utf-8')
        compressed_data = base64.b64decode(base64_data)
        jpg_data = zlib.decompress(compressed_data)

        # Decode the JPEG data
        frame = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), -1)

        detector = load_aruco_detector()

        kalman_filters = {
            '1': initialize_kalman_filter(dim_x=6, dim_z=3),
            '2': initialize_kalman_filter(dim_x=6, dim_z=3),
            '3': initialize_kalman_filter(dim_x=6, dim_z=3),
            '4': initialize_kalman_filter(dim_x=6, dim_z=3),
        }

        frame, markerCorners, markerIds = detect_aruco_markers(frame, detector)

        text_positions = {
            'Marker1': (10, 50),
            'Marker2': (10, 80),
            'Marker3': (10, 110),
            'Marker4': (10, 140),
        }
        
        if not isinstance(markerIds, np.ndarray):
            cv2.imshow('Live Detection', frame)

        elif markerIds is not None:
            mark1_points, mark2_points, mark3_points, mark4_points, mark_weight, weight, height = process_aruco_markers(markerIds, markerCorners)

            W, object_points, marker_id, distance_real, coordinate_relative = estimate_pose(markerCorners, markerIds)

            # Display text in the OpenCV window
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            font_thickness = 1
            font_color = (255, 255, 255)  # White color in BGR
            position = (5, 25)
            text_lines = []
            xrel, yrel, zrel = 0, 0, 0

            for i, marker_id in enumerate(markerIds):

                if 1 <= marker_id <= 4:
                    coord = extract_translation(W[i])
                    measurement = coord  # Assuming 3D measurements

                    # Use moving average
                    kalman_output = calculate_moving_average(marker_id[0], measurement)

                    text_position = text_positions.get(f'Marker{marker_id[0]}', (10, 170))
                    text = (f"Filtered Marker {marker_id} - X: {kalman_output[0]:.2f}, Y: {kalman_output[1]:.2f}, Z: {kalman_output[2]:.2f}")
                    cv2.putText(frame, text, text_position, font, font_scale, font_color, font_thickness)

                else:

                    text_position = (10, 170)
                    global msg_ros
                    if (mark1_points is not None and mark4_points is not None) or (mark2_points is not None and mark3_points is not None):
                        coord = extract_translation(W[1])
                        measurement = coord  # Assuming 3D measurements

                        xrel, yrel, zrel = coordinate_relative
                        #print("Rels: " ,xrel, yrel, zrel)
                        
                        moving_average_output = calculate_moving_average("1", measurement)
                        kalman_output = update_kalman_filter(kalman_filters[str("1"[0])], measurement)
                        
                        xreel = round(float(kalman_output[0]) + xrel, 2)
                        yreel = round(float(kalman_output[1]) + yrel, 2)
                        zreel = round(float(kalman_output[2]) + zrel, 2)

                        text = (f"Filtered Marker {marker_id} - X: {xreel}, Y: {yreel}, Z: {zreel}")
                        cv2.putText(frame, text, text_position, font, font_scale, font_color, font_thickness)
                        
                        msg_ros = f"{height}, {weight}, {xreel}, {yreel}, {zreel} "
                        #print(msg_ros)

                    else:
                        text = (f"Marker {marker_id}, L: X: {xrel}, Y: {yrel}, Z: {zrel}")
                        msg_ros = f"{height}, {weight}, {xrel}, {yrel}, {zrel} "
                        #print(msg_ros)
                        cv2.putText(frame, text, text_position, font, font_scale, font_color, font_thickness)
            

            draw_marker_points(frame, mark1_points, mark2_points, mark3_points, mark4_points, mark_weight, distance_real)

            frame = create_bounding_quadrilateral(mark1_points, mark2_points, mark3_points, mark4_points, frame)

            cv2.imshow('Live Detection', frame)
            # Set the message in the Queue
            msg_queue.put(msg_ros)

            
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit_flag = True

    except UnicodeDecodeError as decode_error:
        print(f"Error decoding payload: {decode_error}")

    except Exception as e:
        print(f"Error decoding and displaying frame: {e}")
        print(type(e).__name__, e)


camera_publisher = rclpy.create_node('camera_publisher')
publisher, timer = create_publisher(camera_publisher, "jekko_camera", msg_ros)


# Set the callback function
client.on_message = on_message

# Subscribe to the topic
client.subscribe(topic)

def mqtt_loop():
    client.loop_forever()

mqtt_thread = threading.Thread(target=mqtt_loop)
mqtt_thread.start()

# Continue with ROS2 loop
rclpy.spin(camera_publisher)

# Wait for the exit flag to be set
while not exit_flag:
    time.sleep(0.1)

# Stop the MQTT loop
client.loop_stop()

# Join the MQTT thread
mqtt_thread.join()