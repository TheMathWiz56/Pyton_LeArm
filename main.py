import LeArm
import Commands
import os

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
    arm = LeArm.Arm()
    command_controller = Commands.Command(arm)

    is_running = True
    while is_running:
        try:
            os.system('cls' if os.name == 'nt' else 'clear')
            print("Type the number of the desired command:")
            print("1) Servo Test")
            print("2) Go To Vertical 0")
            print("3) Go To Stow")
            user_input = input("Select Command: ")

            try:
                user_input_int = int(user_input)
                if user_input_int == 1:
                    command_controller.servo_test()
                elif user_input_int == 2:
                    command_controller.go_to_vertical_0()
                elif user_input_int == 3:
                    command_controller.go_to_stow()
                else:
                    print("Invalid Input")
            except TypeError:
                print("Invalid Input")
        except KeyboardInterrupt:
            print("Exiting")
            is_running = False


if __name__ == '__main__':
    main()
