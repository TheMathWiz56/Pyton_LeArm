import LeArm
import Commands
import os

"""
------------------------
Plan for arm control
------------------------

present Options at start of the program to run simulation or run actual arm

Main.py - has arm.py for setting desired position and returning forward kinematics
    Also read's joysticks and modifies desired position accordingly
LeArm.py - 
class Arm - puts together all of the link instances and in charge of kinematic operations
class link - responsible for keeping track of link parameters
    function homogeneous transform, returns homogeneous transform
class linkParameters - helpful for storing and updating link parameters for a given link
----------------------------------------------------------------------------------------------------------------------

To Do List:
Add db for base angle to movement -- built into xyz scaling
auto point scale? || mode for normal jerky movement and smooth movement
    smooth auto point scales (then pitch?)
check to see how long calculations take -> max frequency
update github branches
change how gripper state changes with movements
errors out when going to the same point as past
"""


def clearTerminal():
    os.system('cls' if os.name == 'nt' else 'clear')


def main():
    arm = LeArm.Arm()
    command_controller = Commands.Command(arm)

    clearTerminal()

    command_controller.initialize_arm()

    is_running = True
    while is_running:
        try:
            print("Type the number of the desired command:")
            print("1) Servo Test")
            print("2) Go To Vertical 0")
            print("3) Go To Stow")
            print("4) Go To Custom Point")
            user_input = input("Select Command: ")

            try:
                user_input_int = int(user_input)
                if user_input_int == 1:
                    command_controller.servo_test()
                elif user_input_int == 2:
                    command_controller.go_to_vertical_0()
                elif user_input_int == 3:
                    command_controller.go_to_stow()
                elif user_input_int == 4:
                    command_controller.go_to()
                else:
                    print("Invalid Input: Not a Command")
            except ValueError:
                print("Invalid Input: Value Error")
        except KeyboardInterrupt:
            print("Exiting")
            is_running = False


if __name__ == '__main__':
    main()
