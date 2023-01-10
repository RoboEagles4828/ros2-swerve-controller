import wpilib
import ctre
import Constants
from Constants import SwervePort
from enum import Enum, auto


class MotorType(Enum):
    RUN_MOTOR = auto()
    TURN_MOTOR = auto()

class SwerveModule():
    
    def __init__(self, swerve_port: SwervePort) -> None:
        self.run_motor = ctre.TalonFX(swerve_port.run_motor_port)
        self.turn_motor = ctre.TalonFX(swerve_port.turn_motor_port)
        self.feedback_device = ctre.TalonFXFeedbackDevice.IntegratedSensor
        self.control_mode = ctre.TalonFXControlMode.Velocity
        self.run_motor.configSelectedFeedbackSensor(self.feedback_device)
        self.turn_motor.configSelectedFeedbackSensor(self.feedback_device)

    
    def setVelocity(self, run_motor_vel, turn_motor_vel):
        self.run_motor.set(self.control_mode, run_motor_vel)
        self.turn_motor.set(self.control_mode, turn_motor_vel)

    def getEncoderInfo(self, type: MotorType):
        if type == MotorType.RUN_MOTOR:
            return {"position": self.run_motor.getSensorCollection().getIntegratedSensorPosition(), "velocity": self.run_motor.getSensorCollection().getIntegratedSensorVelocity()}
        elif type == MotorType.TURN_MOTOR:
            return {"position": self.turn_motor.getSensorCollection().getIntegratedSensorPosition(), "velocity": self.turn_motor.getSensorCollection().getIntegratedSensorVelocity()}

class DriveTrain():
    def __init__(self):
        self.front_left = SwerveModule(Constants.front_left_port)
        self.front_right = SwerveModule(Constants.front_right_port)
        self.back_left = SwerveModule(Constants.back_left_port)
        self.back_right = SwerveModule(Constants.back_right_port)
        self.controller = wpilib.XboxController(Constants.controller_port)
        self.test_motor = ctre.TalonFX(Constants.test_port)

    def setVelocities(self, run_motor_velocities: list, turn_motor_velocities: list):
        #TODO: fix order
        self.front_left.setVelocity(run_motor_velocities[0], turn_motor_velocities[0])
        self.front_right.setVelocity(run_motor_velocities[1], turn_motor_velocities[1])
        self.back_left.setVelocity(run_motor_velocities[2], run_motor_velocities[2])
        self.back_right.setVelocity(run_motor_velocities[3], run_motor_velocities[3])

    def getEncoderInfo(self):
        run_motor_velocities = []
        run_motor_velocities.append(self.front_left.getEncoderInfo(MotorType.RUN_MOTOR)['velocity'])
        run_motor_velocities.append(self.front_right.getEncoderInfo(MotorType.RUN_MOTOR)['velocity'])
        run_motor_velocities.append(self.back_left.getEncoderInfo(MotorType.RUN_MOTOR)['velocity'])
        run_motor_velocities.append(self.back_right.getEncoderInfo(MotorType.RUN_MOTOR)['velocity'])

        turn_motor_velocities = []
        turn_motor_velocities.append(self.front_left.getEncoderInfo(MotorType.TURN_MOTOR)['velocity'])
        turn_motor_velocities.append(self.front_right.getEncoderInfo(MotorType.TURN_MOTOR)['velocity'])
        turn_motor_velocities.append(self.back_left.getEncoderInfo(MotorType.TURN_MOTOR)['velocity'])
        turn_motor_velocities.append(self.back_right.getEncoderInfo(MotorType.TURN_MOTOR)['velocity'])

        run_motor_positions = []
        run_motor_positions.append(self.front_left.getEncoderInfo(MotorType.RUN_MOTOR)['position'])
        run_motor_positions.append(self.front_right.getEncoderInfo(MotorType.RUN_MOTOR)['position'])
        run_motor_positions.append(self.back_left.getEncoderInfo(MotorType.RUN_MOTOR)['position'])
        run_motor_positions.append(self.back_right.getEncoderInfo(MotorType.RUN_MOTOR)['position'])
    
        turn_motor_positions = []
        turn_motor_positions.append(self.front_left.getEncoderInfo(MotorType.TURN_MOTOR)['position'])
        turn_motor_positions.append(self.front_right.getEncoderInfo(MotorType.TURN_MOTOR)['position'])
        turn_motor_positions.append(self.back_left.getEncoderInfo(MotorType.TURN_MOTOR)['position'])
        turn_motor_positions.append(self.back_right.getEncoderInfo(MotorType.TURN_MOTOR)['position'])

        velocities = run_motor_velocities | turn_motor_positions
        positions = run_motor_positions | turn_motor_positions
        return {'position': positions, 'velocity': velocities}

    def setTestVelocity(self, test_velocity):
        self.test_motor.set(ctre.TalonFXControlMode.Velocity, test_velocity)

    def getTestEncoderInfo(self):
        return self.test_motor.getSensorCollection().getIntegratedSensorVelocity(), self.test_motor.getSensorCollection().getIntegratedSensorPosition()

