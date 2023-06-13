import inputs_16btns as inputs
import threading
import numpy as np


device = False

x=0.0
y=0.0
z=0.0
rz=0.0
g=0.0
h=0.0

for dev in inputs.devices:
	#print(dev)
	if "Gamepad" in str(dev):
		device=dev
		print("input device connected")
if not device:
	print("ERROR: input device not found!")



def readJoy():
	global x,y,z,rz,g,h
	while 1:
		if not device:
			return

		events = device.read()
		for event in events:
			if event.code == "ABS_X":
				x=(float(event.state)/128.0)-1.0
			elif event.code == "ABS_Y":
				y=-(float(event.state)/128.0)+1.0
			elif event.code == "ABS_RZ":
				z=-(float(event.state)/128.0)+1.0
			elif event.code == "ABS_Z":
				rz=(float(event.state)/128.0)-1.0
			elif event.code == "BTN_BASE":
				h=float(event.state)
			elif event.code == "BTN_TOP2":
				h=-float(event.state)
			elif event.code == "BTN_BASE2":
				g=float(event.state)
			elif event.code == "BTN_PINKIE":
				g=-float(event.state)
			
			#print(event.code)
			

		#print(x)
		#print(y)
		#print(z)
		#print(rz)
		#print(h)
		#print(g)
		#print("###############")
		
def is_down(i):
	return key_states[i]

def get_key():
	global key_queue
	if len(key_queue)>0:
		return key_queue.pop()
	else:
		return -1

def clear_keys():
	global key_queue
	key_queue=[]

t = threading.Thread(target=readJoy, args=())
t.start()
