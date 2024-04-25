
import LeArm

"""
------------------------
Plan for arm control
------------------------

present Option at start of the program to run simulation or run actual arm (changes how classes are initialized and run)

Main.py - has arm.py for setting desired position and returning forward kinematics
    Also read's joysticks and modifies desired position accordingly
LeArm.py - 
class Arm - puts together all of the link instances and in charge of kinematic operations
class link - responsible for keeping track of link parameters
    function homogeneous transorm, returns homogeneous transform
class linkParameters - helpful for storing and updating link parameters for a given link

To Do List:
Make LinkList and ServoSetpointList Classes for arm's use
Figure out joint angles from given servo angles and update forward kinematics
Basic command for going to a few known positions, vertical 0, horizontal 0, Stow position

Inverse Kinematics implementation process:
start with 3 axis planar inverse kinematics
    with servo movements
    on a controller button press
add base and wrist axes to inverse kinematics

Could make a FSM for handling command flow
"""


def main():
    test = LeArm.Arm()
    print(test.__str__())


if __name__ == '__main__':
    main()
