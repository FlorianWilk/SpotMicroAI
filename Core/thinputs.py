

from time import sleep
from threading import Thread
import queue
import time

from inputs import get_gamepad


class ThreadedInputs:
	NOMATCH = 'No Match'
	
	def __init__(self):
		# Initialise gamepad command dictionary.
		# Add gamepad commands using the append method before executing the start method.
		self.gamepadInputs = {}
		self.lastEventCode = self.NOMATCH
		# Initialise the thread status flag
		self.stopped = False
		self.q = queue.LifoQueue()

	def start(self):
		# Start the thread to poll gamepad event updates
		t = Thread(target=self.gamepad_update, args=())
		t.daemon = True
		t.start()
		
	def gamepad_update(self):
		while True:
			# Should the thread exit?
			if self.stopped:
				return
			# Code execution stops at the following line until a gamepad event occurs.
			events = get_gamepad()
			for event in events:
				event_test = self.gamepadInputs.get(event.code, self.NOMATCH)
				if event_test != self.NOMATCH:
					self.gamepadInputs[event.code] = event.state
					self.lastEventCode = event.code
					self.q.put(event.code)

	def read(self):
		# Return the latest command from gamepad event
		if not self.q.empty():
			newCommand = self.q.get()
			while not self.q.empty():
				trashBin = self.q.get()
	
			return newCommand, self.gamepadInputs[newCommand]
		else:
			return self.NOMATCH, 0

	def stop(self):
		# Stop the game pad thread
		self.stopped = True
		
	def append_command(self, newCommand, newValue):
		# Add new controller command to the list
		if newCommand not in self.gamepadInputs:
			self.gamepadInputs[newCommand] = newValue
		else:
			print('New command already exists')
		
	def delete_command(self, commandKey):
		# Remove controller command from list
		if commandKey in self.gamepadInputs:
			del self.gamepadInputs[commandKey]
		else:
			print('No command to delete')

	def command_value(self, commandKey):
		# Get command value
		if commandKey in self.gamepadInputs:
			return self.gamepadInputs[commandKey]
		else:
			return None
