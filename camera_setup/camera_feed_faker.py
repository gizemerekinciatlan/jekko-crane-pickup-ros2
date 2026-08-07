import cv2
import numpy as np
import paho.mqtt.client as mqtt
import zlib
import base64
import time


print(cv2. __version__ )
# MQTT configuration
broker_address = "mqtt-dashboard.com"
port = 1883
topic = "testtopic/webcam_mqttcast"

# Specify the path to the image file
image_path = "trial_cam.jpg"


client = mqtt.Client()
client.connect(broker_address, port, keepalive=60)
print("Casting starting")

desired_width = 640
desired_height = 480

while True:
    # Read the image file
    frame = cv2.imread(image_path)
    resized_frame = cv2.resize(frame, (desired_width, desired_height))

    cv2.imshow('Resized Frame', resized_frame)

    # Send distorted version
    _, buffer = cv2.imencode('.jpg', resized_frame)
    jpg_data = buffer.tobytes()

    compressed_data = zlib.compress(jpg_data)

    base64_data = base64.b64encode(compressed_data).decode('utf-8')

    client.publish(topic, payload=base64_data, qos=0)

    key = cv2.waitKey(1)
    if key == 27:  # Press 'Esc' to exit
        break

    # Wait for 1 second before sending the next image
    time.sleep(1)

cv2.destroyAllWindows()
