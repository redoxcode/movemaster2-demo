import math
import serial
import serial.tools.list_ports
import time


upper_arm_length = 220 #mm

lower_arm_length = 160 #mm

basis_height = 253 #mm

hand_length = 65 #mm #65 + 142
hand_radial_offset = 0 #mm
hand_radial_offset_rot=0 #deg


base_encoder_0 = 9600
base_encoder_steps = 57600
upper_arm_encoder_0 = 2200
upper_arm_encoder_steps = 57600
lower_arm_encoder_0 = 14400
lower_arm_encoder_steps = -57600
hand_roll_encoder_0 = 2000
hand_roll_encoder_steps = -38400
hand_encoder_0=9700
hand_encoder_steps = -38400

ser=ser = serial.Serial()
calibrated=False


def set_tool(height,radial_offset,offset_rot):
	global hand_length,hand_radial_offset,hand_radial_offset_rot
	hand_length=height+65
	hand_radial_offset=radial_offset
	hand_radial_offset_rot=offset_rot

	

def open_connection(i):
	global ser
	#open serial connection
	ports = serial.tools.list_ports.comports()
	available_ports = []
	for p in ports:
		available_ports.append(p.device)
	print("connecting to port: " + available_ports[i])
	ser = serial.Serial(
		port=available_ports[i],
		baudrate=500000,
	)
	if ser.isOpen():
		ser.close()
	ser.open()
	time.sleep(1) #pause needed to open the port
	while (not ser.isOpen()):
		time.sleep(1)
	print("connected")

def close_connection():
	ser.close()
	
def FK(alphas):
	base_alpha,upper_arm_alpha,lower_arm_alpha,hand_alpha,hand_roll_alpha = alphas
	
	r = math.sin(math.radians(upper_arm_alpha))*upper_arm_length
	z = math.cos(math.radians(upper_arm_alpha))*upper_arm_length
			
	r += math.sin(math.radians(lower_arm_alpha+upper_arm_alpha))*lower_arm_length
	z += math.cos(math.radians(lower_arm_alpha+upper_arm_alpha))*lower_arm_length


	
	roll = hand_roll_alpha
	pitch=upper_arm_alpha+lower_arm_alpha+hand_alpha-90
	z+=basis_height
	z-=math.sin(math.radians(pitch))*hand_length
	r+=math.cos(math.radians(pitch))*hand_length
	
	x = math.cos(math.radians(base_alpha))*r 
	y = math.sin(math.radians(base_alpha))*r 
	
	return [x,y,z,pitch,roll]
		
def IK(pos):
	x,y,z,pitch,roll = pos
	roll = math.radians(roll)
	pitch = math.radians(pitch)
	
	z-=basis_height
	z+=math.sin(pitch)*hand_length
	
	
	r = math.sqrt(pow(x,2)+pow(y,2))
	r-=math.cos(pitch)*hand_length

	span =  math.sqrt(pow(z,2)+pow(r,2))
	if span > (upper_arm_length+lower_arm_length) or span<abs(upper_arm_length-lower_arm_length):
		return(9999,9999,9999,9999,9999)

	lower_arm_alpha = math.pi-math.acos((pow(lower_arm_length,2)+pow(upper_arm_length,2)-pow(span,2))/(2.0*lower_arm_length*upper_arm_length))
	
	upper_arm_alpha = -math.acos((pow(span,2)+pow(upper_arm_length,2)-pow(lower_arm_length,2))/(2*span*upper_arm_length))

	upper_arm_alpha += math.atan2(r,z)
	
	base_alpha = math.atan2(y,x)
	hand_roll_alpha=roll
	hand_alpha = pitch-(upper_arm_alpha+lower_arm_alpha-math.pi*0.5)
	return [math.degrees(base_alpha),math.degrees(upper_arm_alpha),math.degrees(lower_arm_alpha),math.degrees(hand_alpha),math.degrees(hand_roll_alpha)]


def FK_4DOF(alphas):
	base_alpha,upper_arm_alpha,lower_arm_alpha,hand_alpha,hand_roll_alpha = alphas
	
	r = math.sin(math.radians(upper_arm_alpha))*upper_arm_length
	z = math.cos(math.radians(upper_arm_alpha))*upper_arm_length
			
	r += math.sin(math.radians(lower_arm_alpha+upper_arm_alpha))*lower_arm_length
	z += math.cos(math.radians(lower_arm_alpha+upper_arm_alpha))*lower_arm_length

	pitch=upper_arm_alpha+lower_arm_alpha+hand_alpha-90
	
	phi = hand_roll_alpha-base_alpha+hand_radial_offset_rot
	
	z+=basis_height
	z-=math.sin(math.radians(pitch))*hand_length
	r+=math.cos(math.radians(pitch))*hand_length
	
	x = math.cos(math.radians(base_alpha))*r 
	y = math.sin(math.radians(base_alpha))*r 
	
	x+= math.cos(math.radians(phi))*hand_radial_offset
	y-= math.sin(math.radians(phi))*hand_radial_offset
	
	return [x,y,z,phi]

def IK_4DOF(pos):
	x,y,z,phi = pos
	phi = math.radians(phi)
	z-=basis_height
	z+=hand_length
	
	x-= math.cos(phi)*hand_radial_offset
	y+= math.sin(phi)*hand_radial_offset
	
	r = math.sqrt(pow(x,2)+pow(y,2))
	span =  math.sqrt(pow(z,2)+pow(r,2))
	if span > (upper_arm_length+lower_arm_length) or span<abs(upper_arm_length-lower_arm_length):
		return(9999,9999,9999,9999,9999)

	lower_arm_alpha = math.pi-math.acos((pow(lower_arm_length,2)+pow(upper_arm_length,2)-pow(span,2))/(2.0*lower_arm_length*upper_arm_length))
	
	upper_arm_alpha = -math.acos((pow(span,2)+pow(upper_arm_length,2)-pow(lower_arm_length,2))/(2*span*upper_arm_length))

	upper_arm_alpha += math.atan2(r,z)
	
	base_alpha = math.atan2(y,x)
	hand_roll_alpha=phi+base_alpha-math.radians(hand_radial_offset_rot)
	hand_alpha = math.pi-(upper_arm_alpha+lower_arm_alpha)
	return [math.degrees(base_alpha),math.degrees(upper_arm_alpha),math.degrees(lower_arm_alpha),math.degrees(hand_alpha),math.degrees(hand_roll_alpha)]

def is_valid(alphas):
	base_alpha,upper_arm_alpha,lower_arm_alpha,hand_alpha,hand_roll_alpha = alphas
	base_alpha_min=-45
	base_alpha_max=180
	upper_arm_alpha_min = -10
	upper_arm_alpha_max = 120
	lower_arm_alpha_min = 0
	lower_arm_alpha_max = 90
	hand_alpha_max=90
	hand_alpha_min=-90

	test = base_alpha<=base_alpha_max and base_alpha>=base_alpha_min
	test = test and (upper_arm_alpha>=upper_arm_alpha_min and upper_arm_alpha<=upper_arm_alpha_max)
	test = test and (lower_arm_alpha<=lower_arm_alpha_max and lower_arm_alpha>=lower_arm_alpha_min)
	test = test and (hand_alpha<=hand_alpha_max and hand_alpha>=hand_alpha_min)	

	return test


def send_reset():
	global calibrated
	print("starting calibration")
	ser.write(bytes("R;", 'utf-8'))
	time.sleep(1)
	ser.read(1)
	calibrated=True	
	send_alphas([0,30,60,90,0])
	time.sleep(2)
	print("calibration done")



last_send_alphas=[0,0,0,0,0]
def send_alphas(alphas):
	global last_send_alphas
	if calibrated:
		if is_valid(alphas):
			base_alpha,upper_arm_alpha,lower_arm_alpha,hand_alpha,hand_roll_alpha = alphas
			base_encoder = int(base_encoder_0 + base_encoder_steps*(base_alpha/360.0))
			upper_arm_encoder = int(upper_arm_encoder_0 + upper_arm_encoder_steps*(upper_arm_alpha/360.0))
			lower_arm_encoder = int(lower_arm_encoder_0 + lower_arm_encoder_steps*(lower_arm_alpha/360.0))
			hand_encoder = int(hand_encoder_0+hand_encoder_steps*(hand_alpha/360.0))
			hand_roll_encoder = int(hand_roll_encoder_0+hand_roll_encoder_steps*(hand_roll_alpha/360.0))
			
			
			cmd = "T0="+str(base_encoder)+";T1="+str(upper_arm_encoder)+";T2="+str(lower_arm_encoder)+";T3="+str(hand_encoder+hand_roll_encoder)+";T4="+str(hand_encoder-hand_roll_encoder)+";"
			ser.write(bytes(cmd, 'utf-8'))
			last_send_alphas=alphas
		else:
			print("out of range alphas")
			#print(alphas)
	else:
		print("not clibrated")

def gripper(d):
	if d>0.5:
		cmd = "T5=1000;"
		ser.write(bytes(cmd, 'utf-8'))
	elif d<-0.5:
		cmd = "T5=-1000;"
		ser.write(bytes(cmd, 'utf-8')) 
	else:
		cmd = "T5=0;"
		ser.write(bytes(cmd, 'utf-8')) 

def send_pos(pos):
	send_alphas(IK(pos))

def send_pos_4DOF(pos):
	send_alphas(IK_4DOF(pos))
	
def lip_alphas(pos_new,time_frame):
	pos_old = last_send_alphas
	timestep = 0.01
	steps = int(time_frame/timestep)
	pos = [0,0,0,0,0]
	for i in range(steps):
		for j in range(5):
			pos[j] = pos_old[j]+(pos_new[j]-pos_old[j])*(i/steps)
		send_alphas(pos)
		time.sleep(timestep)
	send_alphas(pos_new)
	time.sleep(timestep)
	
def get_pos():
	return FK(last_send_alphas)

def lip(pos_new,time_frame):
	pos_old = get_pos()
	timestep = 0.01
	steps = int(time_frame/timestep)
	pos = [0,0,0,0,0]
	for i in range(steps):
		for j in range(5):
			pos[j] = pos_old[j]+(pos_new[j]-pos_old[j])*(i/steps)
		send_alphas(IK(pos))
		time.sleep(timestep)
	send_alphas(IK(pos_new))
	time.sleep(timestep)
	
def lip_4DOF(pos_new,time_frame):
	pos_old = FK_4DOF(last_send_alphas)
	timestep = 0.01
	steps = int(time_frame/timestep)
	pos = [0,0,0,0]
	for i in range(steps):
		for j in range(4):
			pos[j] = pos_old[j]+(pos_new[j]-pos_old[j])*(i/steps)
		send_alphas(IK_4DOF(pos))
		time.sleep(timestep)
	send_alphas(IK_4DOF(pos_new))
	time.sleep(timestep)

	




