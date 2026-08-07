import cv2
import numpy as np
import paho.mqtt.client as mqtt
import zlib
import base64

# MQTT configuration
broker_address = "mqtt-dashboard.com"
port = 1883
topic = "testtopic/webcam_mqttcast"

cap = cv2.VideoCapture(2) # 2 is for wireless cam / Phone Cam
cap.set(cv2.CAP_PROP_FPS, 1)

client = mqtt.Client()

client.connect(broker_address, port, keepalive=60)
print("casting starting")

while True:
    ret, frame = cap.read()

    if not ret:
        print("Failed to capture frame")
        break

    # Display the original and undistorted frames
    cv2.imshow('Original Frame', frame)

    # Send distorted version
    _, buffer = cv2.imencode('.jpg', frame)
    jpg_data = buffer.tobytes()

    compressed_data = zlib.compress(jpg_data)

    base64_data = base64.b64encode(compressed_data).decode('utf-8')

    client.publish(topic, payload=base64_data, qos=0)
    import time

    # Wait for 1 second
    time.sleep(1)

    key = cv2.waitKey(1)
    if key == 27:  # Press 'Esc' to exit
        break

# Release the camera
cap.release()
cv2.destroyAllWindows()
