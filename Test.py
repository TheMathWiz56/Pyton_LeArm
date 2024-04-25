import os


def main():
    is_running = True
    os.system('cls' if os.name == 'nt' else 'clear')
    while is_running:
        try:
            print("Type the number of the desired command:")
            print("1) Servo Test")
            print("2) Go To Vertical 0")
            print("3) Go To Stow")
            user_input = input("Select Command: ")

            try:
                user_input_int = int(user_input)
                if user_input_int == 1:
                    print("Good Input")
                elif user_input_int == 2:
                    print("Good Input")
                elif user_input_int == 3:
                    print("Good Input")
                else:
                    print("Invalid Input")
            except ValueError:
                print("Invalid Input")
        except KeyboardInterrupt:
            print("Exiting")
            is_running = False


if __name__ == '__main__':
    main()
