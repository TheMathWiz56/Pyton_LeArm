try:
    import Adafruit_PCA9685
except ModuleNotFoundError:
    import adafruit_pca9685
import board
import busio

"""
------------------------
Plan for arm control
------------------------

present Option at start of the program to run simulation or run actual arm (changes how classes are initialized and run)

Main.py - has arm.py for setting desired position and returning forward kinematics
    Also read's joysticks and modifies desired position accordingly
Arm_Kinematics.py - 
class Arm - Puts all the link objects together. Has inverse and forwards kinematic functions and periodically sets servo 
    angles on another thread
class Link- creates transform matrix for that link given link parameters and contains the relevant servo instance
Constants.py - stores all constant values like indexes and offsets

To Do List:
First find offset for each servo and valid range
Joysticks to update inverse kinematics (One joystick changes XY component, DPad does Z and other joystick does wrist, 
    bumper does claw)
create and apply inverse kinematics
    Create smooth pathing function
allow autonomous operations to be stored and run from the commmand line 
"""


def main():
    print("Hello World")


if __name__ == '__main__':
    main()
