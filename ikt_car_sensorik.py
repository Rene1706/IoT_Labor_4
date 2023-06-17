#!/usr/bin/python
import os
from time import time, sleep
import threading
import RPi.GPIO as GPIO
import smbus
import math

GPIO.setmode(GPIO.BCM)

# 1 indicates /dev/i2c-1 (port I2C1)
bus = smbus.SMBus(1)

#################################################################################
# Sensors
#################################################################################

#################################################################################
# Ultrasonic
#################################################################################

class Ultrasonic():
	'''This class is responsible for handling i2c requests to an ultrasonic sensor'''

	def __init__(self,address):
		self.address = address
 	
 	# Aufgabe 2
 	#
	# Diese Methode soll ein Datenbyte an den Ultraschallsensor senden um eine Messung zu starten
	def write(self,value=0x51):
		bus.write_byte_data(self.address, 0x00, value)

	# Aufgabe 2
	#
	# Diese Methode soll den Lichtwert auslesen und zurueckgeben.
	def get_brightness(self):
		brightness = bus.read_byte_data(self.address, 0x01)
		return brightness

	# Aufgabe 2
	#
	# Diese Methode soll die Entfernung auslesen und zurueckgeben. 
	def get_distance(self):
		# Start measurment
		self.write()
		sleep(0.07)

		# Read Registers for distance
		range_high_byte = bus.read_byte_date(self.address, 0x02)
		range_low_byte = bus.read_byte_data(self.address, 0x03)
		print(type(range_high_byte))
		distance = (range_high_byte << 8) + range_low_byte
		return distance

	def get_address(self):
		return self.address

class UltrasonicThread(threading.Thread):
	''' Thread-class for holding ultrasonic data '''

	# distance to obstacle in cm
	distance = 0

	# brightness value
	brightness = 0
	stopped = False
	ultrasonicSensor = None
	# Aufgabe 4
	#
	# Hier muss der Thread initialisiert werden.
	def __init__(self, address):
		threading.Thread.__init__(self)
		self.ultrasonicSensor = Ultrasonic(address)
		self.start()

	# Aufgabe 4
	#
	# Schreiben Sie die Messwerte in die lokalen Variablen
	def run(self):
		while not self.stopped:
			self.distance = self.ultrasonicSensor.get_distance()
			self.brightness = self.ultrasonicSensor.get_brightness()
			
	def stop(self):
		self.stopped = True

#################################################################################
# Compass
#################################################################################

class Compass(object):
	'''This class is responsible for handling i2c requests to a compass sensor'''

	def __init__(self,address):
		self.address = address

	# Aufgabe 2
	#
	# Diese Methode soll den Kompasswert auslesen und zurueckgeben. 
	def get_bearing(self):
		bear_high_byte = bus.read_byte_data(self.address, 2)
		bear_low_byte = bus.read_byte_data(self.address, 3)
		raw = (bear_high_byte << 8) + bear_low_byte
		bearing = raw/10.0
		return bearing

class CompassThread(threading.Thread):
	''' Thread-class for holding compass data '''

	# Compass bearing value
	bearing = 0
	compass = None
	stopped = False
	# Aufgabe 4
	#
	# Hier muss der Thread initialisiert werden.
	def __init__(self, address):
		threading.Thread.__init__(self)
		self.compass = Compass(address)
		self.start()

	# Aufgabe 4
	#
	# Diese Methode soll den Kompasswert aktuell halten.
	def run(self):
		while not self.stopped:
			self.bearing = self.compass.get_bearing()

	def stop(self):
		self.stopped = True

#################################################################################
# Infrared
#################################################################################

class Infrared(object):
	'''This class is responsible for handling i2c requests to an infrared sensor'''

	def __init__(self,address):
		self.address = address
		bus.write_byte(self.address, 0x00, 0x00)
		
	# Aufgabe 2 
	#
	# In dieser Methode soll der gemessene Spannungswert des Infrarotsensors ausgelesen werden.
	def get_voltage(self):
		adc_value = bus.read_byte_data(self.address, 0x00)
		return adc_value/255.0 * 5.0

	# Aufgabe 3
	#
	# Der Spannungswert soll in einen Distanzwert umgerechnet werden.
	def get_distance(self):
		voltage = self.get_voltage()
		distance = voltage/5.0 * 70 + 10
		return distance


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
	# Aufgabe 4
	#
	# Hier muss der Thread initialisiert werden.
	def __init__(self, address, encoder=None):
		threading.Thread.__init__(self)
		self.infrared = Infrared(address)
		self.encoder = encoder
		self.stopped = False
		self.start_distance = self.infrared.get_distance()
		self.start()

	def run(self):
		while not self.stopped:
			self.read_infrared_value()
			self.calculate_parking_space_length()

	# Aufgabe 4
	#
	# Diese Methode soll den Infrarotwert aktuell halten
	def read_infrared_value(self):	
		self.distance = self.infrared.get_distance()

	# Aufgabe 5
	#
	# Hier soll die Berechnung der Laenge der Parkluecke definiert werden
	def calculate_parking_space_length(self):
		# Check if distance has increased more than 3 cm -> next to parking space
		if self.start_distance > self.distance + self.dist_to_recognice_parking:
			if self.in_parking_space == False:
				self.in_parking_space = True
				self.start_travel_distance = self.encoder.get_travel_dist()
			elif self.in_parking_space == True:
				# Update new parking space length as long as we are next to parking space
				self.parking_space_length = self.encoder.get_travel_dist()-self.start_travel_distance
		else:
			if self.in_parking_space == True:
				self.in_parking_space = False
				self.end_travel_distance = self.encoder.get_travel_dist()
				# Final parking space length after end of parking space
				self.parking_space_length = self.end_travel_distance-self.start_travel_distance

	def stop(self):
		self.stopped = True

#################################################################################
# Encoder
#################################################################################
	
class Encoder(object):
	''' This class is resNoneponsible for handling encoder data '''

	# Aufgabe 2
	#
	# Wieviel cm betraegt ein einzelner Encoder-Schritt?
	D = 3 # Durchmesser
	N = 20 # Number of steps
	STEP_LENGTH = math.pi*D/N # in cm

	# number of encoder steps
	count = 0

	def __init__(self, pin=3):
		self.pin = pin
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.pin, GPIO.IN)
		GPIO.add_event_detect(self.pin, GPIO.BOTH, callback=self.count, bouncetime = 1)


	def count(self, channel):
		self.count += 1

	# Aufgabe 2
	#
	# Jeder Flankenwechsel muss zur Berechnung der Entfernung gezaehlt werden. 
	# Definieren Sie alle dazu noetigen Methoden.

	# Aufgabe 2
	# 
	# Diese Methode soll die gesamte zurueckgelegte Distanz zurueckgeben.
	def get_travelled_dist(self):
		return self.count * self.STEP_LENGTH


class EncoderThread(threading.Thread):
	''' Thread-class for holding speed and distance data of all encoders'''

	# current speed.
	speed = 0

	# currently traversed distance.
	distance = 0

	# Aufgabe 4
	#
	# Hier muss der Thread initialisiert werden.
	def __init__(self, encoder):
		threading.Thread.__init__(self)
		self.encoder = encoder
		self.prev_distance = 0
		self.prev_time = time()
		self.stopped = False
		self.start()

	def run(self):
		while not self.stopped:
			self.get_values()

	# Aufgabe 4
	#
	# Diese Methode soll die aktuelle Geschwindigkeit sowie die zurueckgelegte Distanz aktuell halten.
	def get_values(self):
		current_count = self.encoder.count
		current_distance = self.encoder.get_travelled_dist()
		current_time = time()

		elapsed_time = current_time-self.prev_time
		distance_delta = current_distance-self.prev_distance

		if elapsed_time > 0:
			self.speed = distance_delta/elapsed_time
		else:
			self.speed = 0

		self.distance = current_distance
		self.prev_time = current_time
		self.prev_distance = current_distance


	def stop(self):
		self.stopped = True

#################################################################################
# Main Thread
#################################################################################	

if __name__ == "__main__":

	# The GPIO pin to which the encoder is connected
	encoder_pin = 23

	# Aufgabe 1
	#
	# Tragen Sie die i2c Adressen der Sensoren hier ein

	# The i2c addresses of front and rear ultrasound sensors
	ultrasonic_front_i2c_address = 0x70
	ultrasonic_rear_i2c_address = 0x71

	# The i2c address of the compass sensor
	compass_i2c_address = 0x60 

	# The i2c address of the infrared sensor
	infrared_i2c_address = 0x04f # Send 0x00 once in the beginning

	# Aufgabe 6
	#
	# Hier sollen saemtlichen Messwerte periodisch auf der Konsole ausgegeben werden.