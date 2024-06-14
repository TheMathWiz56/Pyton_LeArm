import math as m

import LeArm
from Constants import LeArmConstants


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
        self.le_arm.go_to(1, LeArmConstants.CommandType.FIXED.value, x=0, z=320,
                          pitch=m.pi / 2)

    def go_to(self):
        print(f"""MAX and MIN extensions (mm): with gripper removed:
                MAX: {LeArmConstants.MAX_EXTENSION}
                MIN: {LeArmConstants.MIN_EXTENSION}""")
        x = int(input("Enter the X coordinate: "))
        y = int(input("Enter the Y coordinate: "))
        z = int(input("Enter the Z coordinate: "))
        pitch = int(input("Enter the PITCH (DEGREES): "))
        roll = int(input("Enter the ROLL (DEGREES): "))
        gripper_state = int(input("Enter 0 for gripper open, 1 for gripper middle, 2 for gripper closed: "))
        mode = int(input("Enter 0 for FIXED, 1 for ADJUSTABLE PITCH, 2 for ADJUSTABLE POINT, 3 for STEPPED, "
                         "4 for TRAVEL AT HEIGHT: "))
        self.le_arm.go_to(gripper_state, mode, x, y, z, m.radians(pitch), m.radians(roll))
