class SwervePort():
    def __init__(self):
        self.turn_motor_port: int = None
        self.run_motor_port: int = None

front_left_port = SwervePort()
back_left_port = SwervePort()
front_right_port = SwervePort()
back_right_port = SwervePort()

#TODO: Change ports

front_left_port.run_motor_port = 0
front_left_port.turn_motor_port = 1

back_left_port.run_motor_port = 2
back_left_port.turn_motor_port = 3

front_right_port.run_motor_port = 4
front_right_port.turn_motor_port = 5

back_right_port.run_motor_port = 6
back_right_port.turn_motor_port = 11

test_port = 7

controller_port = 0