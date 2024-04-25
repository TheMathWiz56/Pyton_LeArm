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
        self.le_arm.update_servos_setpoints([LeArmConstants.VERTICAL_POSITIONS_LIST])

    def go_to_stow(self):
        pass
