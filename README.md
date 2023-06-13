# movemaster2-demo
some hastily thrown together code for a robotics demonstration

movemaster2.ino: Firmware that runs on the home made control unit of the MoveMaster2
inputs_16btns.py: Code to get joystick/gamepad events (original code here: https://github.com/zeth/inputs)
InputHandler.py: Handler that uses inputs_16btns.py and keeps track of gamepad inputs
MoveMasterLib.py: Code to use the MoveMaster2 including IK and FK
main.py: Control the MoveMaster2 using the gamepad
