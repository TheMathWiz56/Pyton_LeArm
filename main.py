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
rethink how angles should be passed through -- it is good, honestly
check past_setpoint update -- its happening
check math to make sure there are no mistakes -- naaah


Nice to Have:
compare checks to see if all angles are achievable
FSM to handle command flow
everything good according to pylint

Final Goal:
Have the arm repeatably grab and stack 3D printed boxes (1x1x1 in.)


simulation for arm without raspberry pi

Issues:
Scaling is doing weird things

if a point is not achievable make the current point the past setpoint

Should look into python pointers and more advanced class functionality
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
            # os.system('cls' if os.name == 'nt' else 'clear')
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
                    print("Invalid Input")
            except ValueError:
                print("Invalid Input")
        except KeyboardInterrupt:
            print("Exiting")
            is_running = False


if __name__ == '__main__':
    main()
