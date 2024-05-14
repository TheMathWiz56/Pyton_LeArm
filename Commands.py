import math as m

import LeArm
from Constants import LeArmConstants


def get_gripper_state_from_input(gripper_state):
    return LeArmConstants.GripperState.OPEN.value if gripper_state == 1 else (
        LeArmConstants.GripperState.MIDDLE.value) if gripper_state == 2 else (
        LeArmConstants.GripperState.CLOSED.value)


class Command:
    def __init__(self, le_arm: LeArm.Arm):
        self.le_arm = le_arm
        self.kit = le_arm.get_kit()

    def servo_test(self):
        servo_id = 0
        angle = 90

        is_running = True

        while is_running:
            try:
                self.kit.servo[servo_id].angle = angle
            except KeyboardInterrupt:
                print("Servos Paused")
                try:
                    servo_id = int(input("Enter a servo to move 0-5: "))
                    angle = int(input("Enter an angle 0-180: "))
                except ValueError:
                    print(f"Invalid input, using {servo_id} at angle {angle}")
                except KeyboardInterrupt:
                    is_running = False

    def go_to_vertical_0(self):
        self.le_arm.update_servos_setpoints_raw(LeArmConstants.VERTICAL_POSITIONS_LIST)

    def go_to_stow(self):
        self.le_arm.update_servos_setpoints_raw(LeArmConstants.STOW_POSITIONS_LIST)

    def initialize_arm(self):
        self.le_arm.go_to(LeArmConstants.GripperState.MIDDLE.value, x=0, z=320, pitch=m.pi / 2)

    def go_to(self):
        print(f"""MAX and MIN extensions (mm): with gripper removed:
                MAX: {LeArmConstants.MAX_EXTENSION}
                MIN: {LeArmConstants.MIN_EXTENSION}""")
        x = int(input("Enter the X coordinate: "))
        y = int(input("Enter the Y coordinate: "))
        z = int(input("Enter the Z coordinate: "))
        pitch = int(input("Enter the PITCH coordinate (DEGREES): "))
        roll = int(input("Enter the ROLL coordinate (DEGREES): "))
        gripper_state = int(input("Enter 1 for gripper open, 2 for gripper middle, 3 for gripper closed: "))
        self.le_arm.go_to(get_gripper_state_from_input(gripper_state), x, y, z, m.radians(pitch), m.radians(roll))
