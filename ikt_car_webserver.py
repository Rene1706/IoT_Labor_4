﻿#!/usr/bin/python
import tornado.httpserver
import tornado.ioloop
import tornado.options
import tornado.web
import tornado.websocket
import io
import json
import os

import threading
from ikt_car_sensorik_test import *
#import _servo_ctrl
from math import acos, sqrt, degrees



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
				"brightness": self.ultrasonicThreadFront.brightness,
				"distance": self.ultrasonicThreadFront.distance,
				"rearDistance": self.ultrasonicThreadRear.distance,
				"speed": self.encoderThread.speed
			}
			json_data = {}
			json_data = json.dumps(sensor_data)
			for client in self.clients:
				client.write_message(json_data)

	def stop(self):
		self.stopped = True
		self.ultrasonicThreadFront.stop()
		self.ultrasonicThreadRear.stop()
		self.compassThread.stop()
		self.encoderThread.stop()
		self.infraredThread.stop()

class DrivingThread(threading.Thread):
	'''Thread zum Fahren des Autos'''

	# Einparken
	#
	# Hier muss der Thread initialisiert werden.
	def __init__(self):
		return 0
		
	# Einparken
	#
	# Definieren Sie einen Thread, der auf die ueber den Webserver erhaltenen Befehle reagiert und den Einparkprozess durchfuehrt
	def run(self):
		while not self.stopped:
			continue

	def stop(self):
		self.stopped = True
		

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
		tornado.ioloop.IOLoop.instance().start()
	except KeyboardInterrupt as e:
		dataThread.stop()
		print(e)
		


	# Einparken
	#
	# Erstellen und starten Sie hier eine Instanz des DrivingThread, um das Einparken zu ermoeglichen.

