import time
import InputHandler as joy
import MoveMasterLib as robot

robot.open_connection(1)
robot.set_tool(142,0,0)
robot.send_reset()
time.sleep(1)
now = robot.get_pos()
while 1:
	now = robot.get_pos()
	inp = [joy.x,joy.y,joy.z,joy.h*0.2,joy.rz]

	for i in range(len(now)):
		now[i]+=inp[i]*0.03
	now[2] = max(now[2],25)
	
	print(str(round(now[0]))+" x   "+str(round(now[1]))+" y   "+str(round(now[2]))+" z   "+str(round(now[3]))+" r   "+str(round(now[4]))+" rz")
	
	robot.send_pos(now)
	robot.gripper(joy.g)
	time.sleep(0.001)
