# Dronse
The Boyz Ambulance Drone Code

A server/client scheme to control a drone over the internet. The server code is run on a raspberry pi connected to a pixhawk 4 flight controller over UART. The client can post requests to the server, and the RPi will send commands to the pixhawk to control the drone remotely over the internet.
