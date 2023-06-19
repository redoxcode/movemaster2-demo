import time
import InputHandler as joy
import MoveMasterLib as robot

robot.open_connection(0)
robot.set_tool(142,0,0)
robot.send_reset()
time.sleep(1)
now = robot.get_pos()

pos0 = [280,-191,70,90,-34]

pos1 = [280,0,70,90,0]

pos2 = [208,114,187,90,28]

def pick(pos):
	pos_top = pos.copy()
	pos_top[2]+=80
	robot.lip(pos_top,5)
	robot.gripper(-100)
	time.sleep(1)
	robot.lip(pos,2)
	robot.gripper(100)
	time.sleep(1)
	robot.gripper(joy.g)
	robot.lip(pos_top,2)
	
def place(pos):
	pos_top = pos.copy()
	pos_top[2]+=80
	robot.lip(pos_top,5)
	robot.lip(pos,2)
	robot.gripper(-100)
	time.sleep(1)
	robot.gripper(joy.g)
	robot.lip(pos_top,2)


while 1:
	place(pos0)
	pick(pos0)
	time.sleep(5)
	place(pos1)
	pick(pos1)
	time.sleep(5)
	place(pos2)
	pick(pos2)
	time.sleep(5)
