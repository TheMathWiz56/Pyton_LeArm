"""try:
    import Adafruit_PCA9685
except ModuleNotFoundError:
    import adafruit_pca9685
import board
import busio"""
import Arm_Kinematics

"""
------------------------
Plan for arm control
------------------------

present Option at start of the program to run simulation or run actual arm (changes how classes are initialized and run)

Main.py - has arm.py for setting desired position and returning forward kinematics
    Also read's joysticks and modifies desired position accordingly
Arm_Kinematics.py - 
class Arm - puts together all of the link instances and in charge of kinematic operations
class link - responsible for keeping track of link parameters and assigning outputs to servos
    function homogeneous transorm, returns homogeneous transform
class linkParameters - helpful for storing and updating link parameters for a given link

To Do List:
First find offset for each servo and valid range
Joysticks to update inverse kinematics (One joystick changes XY component, DPad does Z and other joystick does wrist, 
    bumper does claw)
create and apply inverse kinematics
    Create smooth pathing function
allow autonomous operations to be stored and run from the commmand line 

Implement multi-threading at some point to split the kinematic calculations and servo control
"""


def main():
    pass
    test = Arm_Kinematics.Arm()
    print(test.__str__())


if __name__ == '__main__':
    main()
