import LeArm


class Command:
    def __init__(self, le_arm: LeArm.Arm):
        self.le_arm = le_arm
        self.kit = le_arm.get_kit()

    def test_servos(self):
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
        pass
