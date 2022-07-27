# Web streaming example
# Source code from the official PiCamera package
# http://picamera.readthedocs.io/en/latest/recipes2.html#web-streaming

import io
import picamera
import logging
import socketserver
from threading import Condition
from http import server
import cv2
import numpy
import os

# PAGE = ""
# with open('index.html', 'r') as f:
#    PAGE = f.read()


PAGE = """\
<html>

<head>
    <title>COMP5010</title>
</head>

<body>
    <center>
        <h1>Ferret front-facing camera</h1>
        <p>Wave and say hi!</p>
    </center>
    <center><img src="stream.mjpg" width="640" height="480"></center>
</body>

</html>
"""

haarcascade_frontalface_alt_path = (
    "/home/pi/webcam/haarcascade_frontalface_alt.xml"
)
CASCADE = cv2.CascadeClassifier(haarcascade_frontalface_alt_path)

class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b"\xff\xd8"):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                # perform FR
                self.frame = FR(self.buffer.getvalue())

                # self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(301)
            self.send_header("Location", "/index.html")
            self.end_headers()
        elif self.path == "/index.html":
            content = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", 0)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=FRAME"
            )
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except Exception as e:
                logging.warning(
                    "Removed streaming client %s: %s", self.client_address, str(e)
                )
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def FR(buff_val):
    # Convert the picture into a numpy array
    buff = numpy.frombuffer(buff_val, dtype=numpy.uint8)

    if buff.size > 0:

        # Now creates an OpenCV image
        image = cv2.imdecode(buff, 1)

        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Look for faces in the image using the loaded cascade file
        faces = CASCADE.detectMultiScale(gray, 1.1, 5)

        # write number of faces detected to image
        cv2.putText(image, 
            f"{len(faces)} faces found.",   # text
            (20, 30),   # position
            cv2.FONT_HERSHEY_SIMPLEX,   # font
            1,  # font scale
            (0, 0, 0),  # color (black)
            1,  # thickness
            1,  # line type
        )

        # Draw a rectangle around every found face
        for (x, y, w, h) in faces:
            cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 0), 2)

        # encode as JPG
        buff = cv2.imencode(".jpg", image)

        # convert to bytes
        buff = buff[1].tobytes()

        # return modified frame buffer
        return buff

    else:
        return buff_val


with picamera.PiCamera(
    resolution="640x480", framerate=5
) as camera:  # needs a hi-res in order to perform facial detection, but low-fps in order to stay close to realtime
    output = StreamingOutput()
    # Uncomment the next line to change your Pi's Camera rotation (in degrees)
    camera.rotation = 180
    camera.start_recording(output, format="mjpeg")
    try:
        address = ("", 8000)
        server = StreamingServer(address, StreamingHandler)
        server.serve_forever()
    finally:
        camera.stop_recording()
