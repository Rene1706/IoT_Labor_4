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
#from ikt_car_sensorik import *
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
		self.render("index_test.html")

# Aufgabe 3
#
# Der Tornado Webserver muss eine Liste der clients verwalten.  
class WebSocketHandler(tornado.websocket.WebSocketHandler):
	'''Definition der Operationen des WebSocket Servers'''

	print("hello WebSocketHandler")

	def open(self):
		return 0

	def on_message(self, message):
		return 0

	def on_close(self):
		return 0


class DataThread(threading.Thread):
	'''Thread zum Senden der Zustandsdaten an alle Clients aus der Client-Liste'''

	# Aufgabe 3
	#
	# Hier muss der Thread initialisiert werden.
	def __init__(self):
		return 0
	
	# Aufgabe 3
	#
	# Erstellen Sie hier Instanzen von Klassen aus dem ersten Teilversuch		
	def set_sensorik(self, address_ultrasonic_front, address_ultrasonic_back, address_compass, address_infrared, encoder_pin):
		return 0

	# Aufgabe 3
	#
	# Hier muessen die Sensorwerte ausgelesen und an alle Clients des Webservers verschickt werden.
	def run(self):
		while not self.stopped:
			continue

	def stop(self):
		self.stopped = True

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
	# Websocket Server initialization
	tornado.options.parse_command_line()
	app = tornado.web.Application(handlers=[(r"/ws", WebSocketHandler), (r"/", IndexHandler), (r" / ( . * ) ", tornado.web.StaticFileHandler, { "path ": os.path.dirname(__file__)}),])
	httpServer = tornado.httpserver.HTTPServer(app)
	httpServer.listen(options.port)
	print(" Listening on port : ", options.port)
	tornado.ioloop.IOLoop.instance().start()
	# Aufgabe 3
	#
	# Erstellen und starten Sie hier eine Instanz des DataThread und starten Sie den Webserver .


	# Einparken
	#
	# Erstellen und starten Sie hier eine Instanz des DrivingThread, um das Einparken zu ermoeglichen.

