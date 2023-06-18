#!/usr/bin/python
import os
from time import time, sleep
import threading
import math
import random

class UltrasonicThread(threading.Thread):
    ''' Thread-class for holding ultrasonic data '''

    # distance to obstacle in cm
    distance = 0

    # brightness value
    brightness = 0
    stopped = False
    ultrasonicSensor = None

    def __init__(self, address):
        threading.Thread.__init__(self)
        self.start()

    def run(self):
        while not self.stopped:
            # Assign random values to distance and brightness
            self.distance = random.randint(0, 100)
            self.brightness = random.randint(0, 255)
            # Sleep for a short duration to simulate sensor reading delay
            sleep(0.1)
    
    def stop(self):
        self.stopped = True

class CompassThread(threading.Thread):
    ''' Thread-class for holding compass data '''

    # Compass bearing value
    bearing = 0
    direction = "Unknown"
    stopped = False

    def __init__(self, address):
        threading.Thread.__init__(self)
        self.start()

    def run(self):
        while not self.stopped:
            # Assign random values to distance and brightness
            self.bearing = random.randint(0, 100)
            self.direction = random.randint(0, 255)
            # Sleep for a short duration to simulate sensor reading delay
            sleep(0.1)
    
    def stop(self):
        self.stopped = True

class InfraredThread(threading.Thread):
    ''' Thread-class for holding Infrared data '''

	# distance to an obstacle in cm
    distance = 0
    # length of parking space in cm
    parking_space_length = 0
    dist_to_recognice_parking = 3
    start_distance = 0
    start_travel_distance = 0
    end_travel_distance = 0
    in_parking_space = False
    stopped = False

    def __init__(self, address, encoder=None):
        threading.Thread.__init__(self)
        self.start()

    def run(self):
        while not self.stopped:
            # Assign random values to distance and brightness
            self.parking_space_length = random.randint(0, 100)
            self.distance = random.randint(0, 255)
            # Sleep for a short duration to simulate sensor reading delay
            sleep(0.1)
    
    def stop(self):
        self.stopped = True

class Encoder(object):
	''' This class is resNoneponsible for handling encoder data '''
	count = 0

	def __init__(self, pin=23):
		self.pin = pin

class EncoderThread(threading.Thread):
    ''' Thread-class for holding speed and distance data of all encoders'''

	# current speed.
    speed = 0

	# currently traversed distance.
    distance = 0
    stopped = False

    def __init__(self, encoder):
        threading.Thread.__init__(self)
        self.encoder = encoder
        self.start()

    def run(self):
        while not self.stopped:
            # Assign random values to distance and brightness
            self.speed = random.randint(0, 100)
            self.distance = random.randint(0, 255)
            # Sleep for a short duration to simulate sensor reading delay
            sleep(0.1)
    
    def stop(self):
        self.stopped = True