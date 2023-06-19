#!/usr/bin/python
import tornado.httpserver
import tornado.ioloop
import tornado.options
import tornado.web
import tornado.websocket
import io
import json
import os

import threading
from ikt_car_sensorik import *
from servo_ctrl import Motor, Steering
from math import acos, sqrt, degrees
import time


# Parking
parking_event = threading.Event()

# Aufgabe 4
#
# Der Tornado Webserver soll die Datei index.html am Port 8081 zur Verfügung stellen
# Der Server bietet eine Datei index.html am Port 8081 an.
from tornado.options import define, options
define("port", default=8081, help="run on the given port ", type=int)

class IndexHandler(tornado.web.RequestHandler):
	@tornado.web.asynchronous
	def get(self):
		self.render("index.html")

# Aufgabe 3
#
# Der Tornado Webserver muss eine Liste der clients verwalten.  
class WebSocketHandler(tornado.websocket.WebSocketHandler):
	'''Definition der Operationen des WebSocket Servers'''

	print("hello WebSocketHandler")

	def open(self):
		if self not in clients:
			print("new connection: {}".format(self.request.remote_ip))
			clients.append(self)

	def on_message(self, message):
		print("message received {}".format(message))
		print(type(message))
		print(str(message))
		if str(message) == "Start parking":
			print("Event is set")
			parking_event.set()
		if str(message) == "Stop parking":
			parking_event.clear()
		#json_message = {}
		#json_message["response"] = message
		#json_message = json.dumps(json_message)
		#self.write_message(json_message)

	def on_close(self):
		if self in clients:
			print("connection closed")
			clients.remove(self)


class DataThread(threading.Thread):
	'''Thread zum Senden der Zustandsdaten an alle Clients aus der Client-Liste'''
	parking_time = 0.0
	parking_distance = 0.0
	# Aufgabe 3
	#
	# Hier muss der Thread initialisiert werden.
	def __init__(self, clients):
		threading.Thread.__init__(self)
		self.clients = clients
		self.stopped = False
	
	# Aufgabe 3
	#
	# Erstellen Sie hier Instanzen von Klassen aus dem ersten Teilversuch		
	def set_sensorik(self, address_ultrasonic_front, address_ultrasonic_back, address_compass, address_infrared, encoder_pin):
		self.ultrasonicThreadFront = UltrasonicThread(address_ultrasonic_front)
		self.ultrasonicThreadRear = UltrasonicThread(address_ultrasonic_back)
		self.compassThread = CompassThread(address_compass)

		encoder = Encoder(encoder_pin)
		self.encoderThread = EncoderThread(encoder)
		self.infraredThread = InfraredThread(address_infrared, encoder)
	

	# Aufgabe 3
	#
	# Hier muessen die Sensorwerte ausgelesen und an alle Clients des Webservers verschickt werden.
	def run(self):
		while not self.stopped:
			sensor_data = {
				"parkingSpaceLength": self.infraredThread.parking_space_length,
				"distance": self.infraredThread.distance,
				"bearing": self.compassThread.bearing,
				"speed": self.encoderThread.speed,
				"parkingTime": self.parking_time,
				"parkingDistance": self.parking_distance
			}
			json_data = {}
			json_data = json.dumps(sensor_data)
			for client in self.clients:
				client.write_message(json_data)
			sleep(0.2)

	def stop(self):
		self.stopped = True
		self.ultrasonicThreadFront.stop()
		self.ultrasonicThreadRear.stop()
		self.compassThread.stop()
		self.encoderThread.stop()
		self.infraredThread.stop()

class DrivingThread(threading.Thread):
	'''Thread zum Fahren des Autos'''
	min_parking_length = 99999999.9
	parking_phases = {"drive_back": True, "turn_right": False, "turn_left": False, "check_alignment": False}
	
	# Einparken
	#
	# Hier muss der Thread initialisiert werden.
	def __init__(self, dataThread):
		threading.Thread.__init__(self)
		self.dataThread = dataThread
		self.stopped = False
		self.start_time = None
		self.parking_driven_length_start = None
                # Initialisierung Motor und steering object 
                self.motor = Motor(1)
                self.steering = Steering(2)
                # Use Object function to reset speed/angle
                self.motor.set_speed(0)
                self.steering.set_angle(0)
                self.iteration_counter = 0
		
	# Einparken
	#
	# Definieren Sie einen Thread, der auf die ueber den Webserver erhaltenen Befehle reagiert und den Einparkprozess durchfuehrt
	def run(self):
		while not self.stopped:
			if parking_event.is_set():
                                self.motor.set_speed(10)
                                #self.steering.set_angle(45)
                                #sleep(3)
                                #self.motor.set_speed(0)
                                #self.steering.set_angle(0)
                                #sleep(3)
				# Check/Update parking timer
				if not self.start_time:
					self.start_time = time.time()
				elapsed_time = time.time() - self.start_time
				self.dataThread.parking_time = float(round(elapsed_time,2))

				# Check/update parking_driven_length
				if not self.parking_driven_length_start:
					self.parking_driven_length_start = self.dataThread.encoderThread.distance
				driven_distance = self.dataThread.encoderThread.distance - self.parking_driven_length_start
				self.dataThread.parking_distance = float(round(driven_distance, 2))
				if self.iteration_counter < 20:
                                        self.motor.set_speed(4)
                                elif self.iteration_counter < 25:
                                        self.motor.set_speed(0)
                                elif self.iteration_counter < 40:
                                        self.steering.set_angle(45)
                                        self.motor.set_speed(-3)
                                elif self.iteration_counter < 50:
                                        self.steering.set_angle(0)
                                elif self.iteration_counter < 60:
                                        self.steering.set_angle(-45)
                                elif self.iteration_counter < 70:
                                        self.motor.set_speed(0)
                                        self.steering.set_angle(0)
                                self.iteration_counter += 1
			else:
				# Reset parking timer
                                self.motor.set_speed(0)
				self.start_time = None
				self.dataThread.parking_time = 0.0
			        # Reset parking driven length
				self.parking_driven_length_start = None
				self.dataThread.parking_distance = 0.0
                        sleep(0.1)

	def stop(self):
                self.stopped = True
		parking_event.clear()
		self.dataThread.stop()
		print("Parking thred stopped")

	def get_min_parking_length(self):
		w = 3.0 #TODO get car width
		p = self.dataThread.infraredThread.distance
		f = 3.0 #TODO get distance back axial to front
		r = 2.0 #TODO get effective tourn radius
		b = 2.0 #TODO get distance back axial to back

		l = sqrt(2*r*w+f**2+b)
		self.min_parking_length = l

	def automate_parking(self):
		# 1. Mit Heck bis ans heck des autos (distance > p)
		# 2. Rechts einlenken max
		# 3. Bestimmte strecke fahren
		# 4. Links einlenken max
		# 5. Halten wenn hinten fast berührt 
		# 6. Checken ob compass anfang = compass ende (parallel ?? Lenkwinkel??)
		# 7. 
		#if self.parking_phases["drive_back"] == True:
		#	if self.dataThread.infraredThred.distance > 
		pass

		
		

if __name__ == "__main__":
	print("Main Thread started")
	clients = []

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

	try:
		# Websocket Server initialization
		tornado.options.parse_command_line()
		app = tornado.web.Application(handlers=[(r"/ws", WebSocketHandler), (r"/", IndexHandler), (r" / ( . * ) ", tornado.web.StaticFileHandler, { "path ": os.path.dirname(__file__)}),])
		httpServer = tornado.httpserver.HTTPServer(app)
		httpServer.listen(options.port)
		print("Listening on port ", options.port)
		# Aufgabe 3
		#
		# Erstellen und starten Sie hier eine Instanz des DataThread und starten Sie den Webserver .
		dataThread = DataThread(clients)
		dataThread.set_sensorik(ultrasonic_front_i2c_address, ultrasonic_rear_i2c_address, compass_i2c_address, infrared_i2c_address, encoder_pin)
		dataThread.start()
		drivingThread = DrivingThread(dataThread)
		drivingThread.start()



		tornado.ioloop.IOLoop.instance().start()
	except KeyboardInterrupt as e:
		dataThread.stop()
		drivingThread.stop()
		print(e)
		


	# Einparken
	#
	# Erstellen und starten Sie hier eine Instanz des DrivingThread, um das Einparken zu ermoeglichen.

