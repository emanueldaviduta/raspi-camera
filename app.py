

from fastapi import FastAPI, Response
from fastapi.responses import HTMLResponse, StreamingResponse
from picamera2 import Picamera2
import time
import io
import uvicorn
from PIL import Image
import cv2
import libcamera
import Adafruit_DHT
import RPi.GPIO as GPIO
import threading
from datetime import datetime
from gpiozero import CPUTemperature
import socket

#FAST Api Zone
app = FastAPI()
app.motion_detected = None

#Pi Camera Zone
picam2 = Picamera2()
camera_config = picam2.create_still_configuration(
	main={"size": (640, 480), "format": "BGR888"},
	transform=None
)
camera_config["transform"] = libcamera.Transform(hflip=1, vflip=0, transpose=0)
picam2.configure(camera_config)
picam2.start()

#Movement Senzor zone
GPIO.setmode(GPIO.BCM)
PIR_PIN = 23

GPIO.setup(PIR_PIN, GPIO.IN)


def monitor_motion():
	try:
		while True:
			if GPIO.input(PIR_PIN):
				app.motion_detected = datetime.now()
			time.sleep(1)
	finally:
		GPIO.cleanup()

thread = threading.Thread(target=monitor_motion, daemon=True)
thread.start()

#Temperature zone
sensor = Adafruit_DHT.DHT11
gpio_pin = 18
humidity, temperature = Adafruit_DHT.read_retry(sensor, gpio_pin)


@app.get("/live", response_class=HTMLResponse)
async def index(shutter_speed:int = 0, gain:float = 0.0,code: str="TH01"):
	last_motion_str="No motion detected yet"
	if app.motion_detected:
		time_diff = datetime.now() - app.motion_detected
		diff_sec = str(round(time_diff.seconds % 60, 0)) + "sec"
		diff_min = str(round(time_diff.seconds / 60, 0)) + "min"
		last_motion_str = diff_min + diff_sec  + " ago"
		last_motion_str += str(time_diff.seconds)
	cpu = CPUTemperature()
	cpu_temp = cpu.temperature
	video_html = f"<img class=\"camera\" src=\"/video_feed?shutter_speed={shutter_speed}&gain={gain}\" >"
	#video_html = "<video autoplay playsinline class=\"camera\"><source src=\"/video_feed\" type=\"video/mp4\"></video>"
	if code != "TH01":
		video_html = "<h1>Cod de acces necesar</h1>"
	HTML_PAGE =  f"""
<!DOCTYPE html>
<html>
	<head>
		<title>Camera Preview</title>
		<meta http-echiv="refresh" content="5">
		<style>
			body {{ font-family: Arial, sans-serif; }}
			.flex {{ display: flex; flex-direction:column }}
			.camera {{ margin: auto; width: 100%; height: calc(100vw * 0.75); max-width: 1200px; max-height: 800px; }}
			.center {{ text-align: center }}
		</style>
	</head>
	<body class="flex">
		<h1 class="center">Raspberry Pi Camera Preview</h1>
		{video_html}
		<div class="center">
			<p>Temperature: {temperature}C</p>
			<p>Humidity: {humidity} %</p>
			<p>Last motion: {last_motion_str}</p>
			<hr>
			<p>CPU: {cpu_temp}</p>
		</div>
	</body>
</html
"""

	return HTML_PAGE

@app.get("/video_feed")
async def video_feed(shutter_speed: int, gain:float):
	def generate(ss: int, gg:float):
		while True:
			if ss != 0:
				picam2.controls.ExposureTime = ss*100000
			if gg != 0.0:
				picam2.controls.AnalogueGain = gg

			frame = picam2.capture_array()
			image = Image.fromarray(frame)
			buf = io.BytesIO()
			image.save(buf, format="JPEG")
			buf.seek(0)

			yield (
				b"--frame\r\n"
				b"Content-Type: image/jpeg\r\n\r\n" + buf.read() + b"\r\n"
			)
			time.sleep(0.1)
	return StreamingResponse(generate(shutter_speed, gain), media_type="multipart/x-mixed-replace; boundary=frame")

@app.on_event("shutdown")
def clean_gpio():
	GPIO.cleanup()

def wait_for_network(timeout=60, interval=5):
    """Wait for network connection to be available."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            # Attempt to connect to a reliable host (Google's public DNS)
            socket.create_connection(("8.8.8.8", 53), timeout=2)
            print("Network connection established.")
            return True
        except (socket.timeout, socket.error):
            print("Waiting for network connection...")
            time.sleep(interval)
    print("Network connection not available within the timeout period.")
    return False

if __name__ == "__main__":
	if wait_for_network():
		uvicorn.run(app, host="0.0.0.0", port=8000)

