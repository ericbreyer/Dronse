# Web streaming example
# Source code from the official PiCamera package
# http://picamera.readthedocs.io/en/latest/recipes2.html#web-streaming

import io
import logging

import picamera

import socketserver
from threading import Condition
from http import server
from http.server import BaseHTTPRequestHandler, HTTPServer

from dronekit import connect, Command, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time, sys, argparse, math

import json

#vehicle = connect('/dev/serial0', wait_ready=False, baud=57600)

#HELPER FUNCTIONS
def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation;

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = "GUIDED"
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
    vehicle.mode = "LOITER"




class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)

class ServerHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path =='/data':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write("GPS: {}".format(vehicle.location.global_frame).encode('utf-8'))
            self.wfile.write("<br/>Heading: {}".format(vehicle.heading).encode('utf-8'))
            self.wfile.write("<br/><br/>\nMode: {}".format(vehicle.mode.name).encode('utf-8'))
            self.wfile.write("<br/><br/>Battery: {}".format(vehicle.battery).encode('utf-8'))
        elif self.path == '/index.html':
            with open("index.html","r") as f:
                content = f.read().encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.send_header('Content-Length', len(content))
                self.end_headers()
                self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()
    def do_POST(self):
        # Gets the size of incoming data
        content_length = int(self.headers['Content-Length']) 
        # Gets the data itself
        post_data = self.rfile.read(content_length)
        # log the request
        logging.info("POST request,\nPath: %s\nHeaders:\n%s\n\nBody:\n%s\n",
                str(self.path), str(self.headers), post_data.decode('utf-8'))
        
        if self.path == '/rotate':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            rotate()
            
        elif self.path == '/BRAKE':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            vehicle.mode = "BRAKE"
            
        elif self.path == '/RTL':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            vehicle.mode = "RTL"
            
        elif self.path == '/AUTO':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            vehicle.mode = "AUTO"
           
        elif self.path =="/ARM_AND_TAKEOFF":
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            arm_and_takeoff(10)
            
        elif self.path =="/doGuided":
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            
            directions = json.loads(post_data)
            
            for k, v in directions.items():
                directions[k] = int(v)
            
            print(vehicle.location.global_relative_frame)

            print(get_location_metres(vehicle.location.global_relative_frame, directions["North"],directions["East"]))
            
            vehicle.mode = "GUIDED"
            
            vehicle.simple_goto(get_location_metres(vehicle.location.global_relative_frame, directions["North"],directions["East"]))
            
        
        self.wfile.write("POST request for {}".format(self.path).encode('utf-8'))

        
class Server(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

with picamera.PiCamera(resolution='64x64', framerate=5) as camera:
    def rotate():
        camera.rotation += 90;
    output = StreamingOutput()
    #Uncomment the next line to change your Pi's Camera rotation (in degrees)
    #camera.rotation = 90
    camera.start_recording(output, format='mjpeg')
    try:
        address = ('', 8000)
        server = Server(address, ServerHandler)
        server.serve_forever()
    finally:
        camera.stop_recording()

