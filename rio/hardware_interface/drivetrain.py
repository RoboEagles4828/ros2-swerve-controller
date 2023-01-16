import wpilib
import ctre
from enum import Enum, auto
import math
   

class SwervePort():
    def __init__(self):
        self.turn_motor_port: int = None
        self.run_motor_port: int = None

class Constants:
    front_left_port = SwervePort()
    back_left_port = SwervePort()
    front_right_port = SwervePort()
    back_right_port = SwervePort()
    test_port = SwervePort()

    #TODO: change CAN ids
    front_left_port.run_motor_port = 0
    front_left_port.turn_motor_port = 1

    back_left_port.run_motor_port = 2
    back_left_port.turn_motor_port = 3

    front_right_port.run_motor_port = 4
    front_right_port.turn_motor_port = 5

    back_right_port.run_motor_port = 12
    back_right_port.turn_motor_port = 11

    controller_port = 0

    test_port.run_motor_port = 6
    test_port.turn_motor_port = 7

class MotorType(Enum):
    RUN_MOTOR = auto()
    TURN_MOTOR = auto()

class SwerveModule():
    
    def __init__(self, swerve_port: SwervePort) -> None:
        self.run_motor = ctre.TalonFX(swerve_port.run_motor_port)
        self.turn_motor = ctre.TalonFX(swerve_port.turn_motor_port)

        self.run_motor.configFactoryDefault()
        self.run_motor.configNeutralDeadband(0.001)
        self.run_motor.configSelectedFeedbackSensor(ctre.TalonFXFeedbackDevice.IntegratedSensor, 0, 30)
        self.run_motor.configNominalOutputForward(0, 30)
        self.run_motor.configNominalOutputReverse(0, 30)
        self.run_motor.configPeakOutputForward(1, 30)
        self.run_motor.configPeakOutputReverse(-1, 30)
        self.kf = 1023.0/20660.0
        self.kp = 0.1
        self.ki = 0.001
        self.kd = 5
        self.run_motor.config_kF(0, self.kf, 30)
        self.run_motor.config_kP(0, self.kp, 30)
        self.run_motor.config_kI(0, self.ki, 30)
        self.run_motor.config_kD(0, self.kd, 30)

        self.turn_motor.configFactoryDefault()
        self.turn_motor.configNeutralDeadband(0.001)
        self.turn_motor.configSelectedFeedbackSensor(ctre.TalonFXFeedbackDevice.IntegratedSensor, 0, 30)
        self.turn_motor.configNominalOutputForward(0, 30)
        self.turn_motor.configNominalOutputReverse(0, 30)
        self.turn_motor.configPeakOutputForward(1, 30)
        self.turn_motor.configPeakOutputReverse(-1, 30)
        self.turn_motor.config_kF(0, self.kf, 30)
        self.turn_motor.config_kP(0, self.kp, 30)
        self.turn_motor.config_kI(0, self.ki, 30)
        self.turn_motor.config_kD(0, self.kd, 30)

    
    def setVelocity(self, run_motor_vel, turn_motor_vel):
        TICKS_PER_REV = 2048
        TICKS_PER_RAD = TICKS_PER_REV / (2 * math.pi)
        scaled_vel = TICKS_PER_RAD * run_motor_vel / 10.0
        scaled_vel2 = TICKS_PER_RAD * turn_motor_vel / 10.0

        self.run_motor.set(ctre.TalonFXControlMode.Velocity, scaled_vel)
        self.turn_motor.set(ctre.TalonFXControlMode.Velocity, scaled_vel2)

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
        self.test = SwerveModule(Constants.test_port)
        self.controller = wpilib.XboxController(Constants.controller_port)

        # self.test_motor = ctre.TalonFX(Constants.test_port)
        # self.test_motor.configFactoryDefault()
        # self.test_motor.configNeutralDeadband(0.001)
        # self.test_motor.configSelectedFeedbackSensor(ctre.TalonFXFeedbackDevice.IntegratedSensor, 0, 30)
        # self.test_motor.configNominalOutputForward(0, 30)
        # self.test_motor.configNominalOutputReverse(0, 30)
        # self.test_motor.configPeakOutputForward(1, 30)
        # self.test_motor.configPeakOutputReverse(-1, 30)
        # self.kf = 1023.0/20660.0
        # self.kp = 0.1
        # self.ki = 0.001
        # self.kd = 5
        # self.test_motor.config_kF(0, self.kf, 30)
        # self.test_motor.config_kP(0, self.kp, 30)
        # self.test_motor.config_kI(0, self.ki, 30)
        # self.test_motor.config_kD(0, self.kd, 30)

        # self.test_motor_2 = ctre.TalonFX(Constants.test_port_2)
        # self.test_motor_2.configFactoryDefault()
        # self.test_motor_2.configNeutralDeadband(0.001)
        # self.test_motor_2.configSelectedFeedbackSensor(ctre.TalonFXFeedbackDevice.IntegratedSensor, 0, 30)
        # self.test_motor_2.configNominalOutputForward(0, 30)
        # self.test_motor_2.configNominalOutputReverse(0, 30)
        # self.test_motor_2.configPeakOutputForward(1, 30)
        # self.test_motor_2.configPeakOutputReverse(-1, 30)
        # self.test_motor_2.config_kF(0, self.kf, 30)
        # self.test_motor_2.config_kP(0, self.kp, 30)
        # self.test_motor_2.config_kI(0, self.ki, 30)
        # self.test_motor_2.config_kD(0, self.kd, 30)

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

        velocities = run_motor_velocities + turn_motor_positions
        positions = run_motor_positions + turn_motor_positions
        return {'position': positions, 'velocity': velocities}

    def setTestVelocity(self, test_velocity, test_velocity2):
        # TICKS_PER_REV = 2048
        # TICKS_PER_RAD = TICKS_PER_REV / (2 * math.pi)
        # scaled_vel = TICKS_PER_RAD * test_velocity / 10.0
        # scaled_vel2 = TICKS_PER_RAD * test_velocity2 / 10.0
        # self.test_motor.set(ctre.TalonFXControlMode.PercentOutput, test_velocity)
        # self.test_motor_2.set(ctre.TalonFXControlMode.PercentOutput, test_velocity2)
        self.test.setVelocity(test_velocity, test_velocity2)

    def getTestEncoderInfo(self):
        return self.test_motor.getSensorCollection().getIntegratedSensorPosition(), self.test_motor.getSensorCollection().getIntegratedSensorVelocity()

