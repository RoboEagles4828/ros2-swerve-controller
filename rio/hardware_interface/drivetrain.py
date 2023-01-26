import wpilib
import ctre
from enum import Enum, auto
import math
   

class SwervePort():
    def __init__(self):
        self.axle_motor_port: int = None
        self.wheel_motor_port: int = None

class Constants:


    #TODO: change CAN ids
    front_left_wheel_motor_port = 9
    front_left_axle_motor_port = 7

    back_left_wheel_motor_port = 6
    back_left_axle_motor_port = 4

    front_right_wheel_motor_port = 12
    front_right_axle_motor_port = 10

    back_right_wheel_motor_port = 3
    back_right_axle_motor_port = 1

    controller_port = 0

    test_wheel_motor_port = 6
    test_axle_motor_port = 7

    WHEEL_JOINT_GEAR_RATIO = 6.75 #8.14
    AXLE_JOINT_GEAR_RATIO = 150.0/7.0
    TICKS_PER_REV = 2048
    TICKS_PER_RAD = TICKS_PER_REV / (2 * math.pi)


class MotorType(Enum):
    wheel_motor = auto()
    axle_motor = auto()

class SwerveModule():
    
    def __init__(self, axle_joint_name, axle_joint_port, wheel_joint_name, wheel_joint_port) -> None:
        self.wheel_motor = ctre.TalonFX(wheel_joint_port)
        self.axle_motor = ctre.TalonFX(axle_joint_port)
        self.wheel_joint_name = wheel_joint_name
        self.axle_joint_name = axle_joint_name
        self.wheel_joint_port = wheel_joint_port
        self.axle_joint_port = axle_joint_port

        self.kf = 1023.0/20660.0
        self.kp = 0.1
        self.ki = 0.001
        self.kd = 5

        self.wheel_motor.configFactoryDefault()
        self.wheel_motor.configNeutralDeadband(0.001)
        self.wheel_motor.configSelectedFeedbackSensor(ctre.TalonFXFeedbackDevice.IntegratedSensor, 0, 30)
        self.wheel_motor.configNominalOutputForward(0, 30)
        self.wheel_motor.configNominalOutputReverse(0, 30)
        self.wheel_motor.configPeakOutputForward(1, 30)
        self.wheel_motor.configPeakOutputReverse(-1, 30)
        self.wheel_motor.config_kF(0, self.kf, 30)
        self.wheel_motor.config_kP(0, self.kp, 30)
        self.wheel_motor.config_kI(0, self.ki, 30)
        self.wheel_motor.config_kD(0, self.kd, 30)

        self.axle_motor.configFactoryDefault()
        self.axle_motor.configNeutralDeadband(0.001)
        self.axle_motor.configSelectedFeedbackSensor(ctre.TalonFXFeedbackDevice.IntegratedSensor, 0, 30)
        self.axle_motor.configNominalOutputForward(0, 30)
        self.axle_motor.configNominalOutputReverse(0, 30)
        self.axle_motor.configPeakOutputForward(1, 30)
        self.axle_motor.configPeakOutputReverse(-1, 30)
        self.axle_motor.config_kF(0, self.kf, 30)
        self.axle_motor.config_kP(0, self.kp, 30)
        self.axle_motor.config_kI(0, self.ki, 30)
        self.axle_motor.config_kD(0, self.kd, 30)


    def convertToTicks(self, angular_vel):
        scaled_vel = Constants.TICKS_PER_RAD * angular_vel / 10.0

        return scaled_vel
 
    def scaleAxleVelocity(self, ticks_per_100ms):
        scaled_vel = ticks_per_100ms * Constants.AXLE_JOINT_GEAR_RATIO
        return scaled_vel

    def scaleWheelVelocity(self, ticks_per_100ms):
        scaled_vel = ticks_per_100ms * Constants.WHEEL_JOINT_GEAR_RATIO
        return scaled_vel

    def setVelocities(self, wheel_motor_vel, axle_motor_vel):
        print(f'WHEEL_NAME - {self.wheel_joint_name}, WHEEL_PORT - {self.wheel_joint_port}')
        print(f'SETTING VELOCITIES: WHEEL - {wheel_motor_vel}, AXLE - {axle_motor_vel}')
        wheel_vel = self.convertToTicks(self.scaleWheelVelocity(wheel_motor_vel - axle_motor_vel/1.9)) #(2-(wheel_motor_vel/(66*math.pi)))
        axle_vel = self.convertToTicks(self.scaleAxleVelocity(axle_motor_vel))
        self.wheel_motor.set(ctre.TalonFXControlMode.Velocity, wheel_vel)
        self.axle_motor.set(ctre.TalonFXControlMode.Velocity, axle_vel)

    def stop(self):
        self.wheel_motor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        self.axle_motor.set(ctre.TalonFXControlMode.PercentOutput, 0)

    def convertToRadians(self, ticks):
        return ticks * 10 / Constants.TICKS_PER_RAD

    def getEncoderInfo(self):
            output = \
            {
                "wheel_joint": 
                    {
                        "name": self.wheel_joint_name,
                        "position": self.convertToRadians(self.wheel_motor.getSensorCollection().getIntegratedSensorPosition()) % math.pi, 
                        "velocity": self.convertToRadians(self.wheel_motor.getSensorCollection().getIntegratedSensorVelocity())
                    },
                "axle_joint":
                    {
                        "name": self.axle_joint_name,
                        "position": self.convertToRadians(self.axle_motor.getSensorCollection().getIntegratedSensorPosition()) % math.pi,
                        "velocity": self.convertToRadians(self.axle_motor.getSensorCollection().getIntegratedSensorVelocity())
                    }
            }
            return output
            # return {"position": self.axle_motor.getSensorCollection().getIntegratedSensorPosition(), "velocity": self.axle_motor.getSensorCollection().getIntegratedSensorVelocity()}

class DriveTrain():
    def __init__(self):
        self.front_left = SwerveModule("front_left_axle_joint", Constants.front_left_axle_motor_port, "front_left_wheel_joint", Constants.front_left_wheel_motor_port)
        self.front_right = SwerveModule("front_right_axle_joint", Constants.front_right_axle_motor_port, "front_right_wheel_joint", Constants.front_right_wheel_motor_port)
        self.back_left = SwerveModule("rear_left_axle_joint", Constants.back_left_axle_motor_port, "rear_left_wheel_joint", Constants.back_left_wheel_motor_port)
        self.back_right = SwerveModule("rear_right_axle_joint", Constants.back_right_axle_motor_port, "rear_right_wheel_joint", Constants.back_right_wheel_motor_port)

        self.controller = wpilib.XboxController(Constants.controller_port)
        # self.joints = \
        # {
        #     'front_left_wheel_joint': self.front_left.wheel_motor,
        #     'front_left_axle_joint': self.front_left.axle_motor,
        #     'front_right_wheel_joint': self.front_right.wheel_motor,
        #     'front_right_axle_joint': self.front_right.axle_motor,
        #     'rear_left_wheel_joint': self.back_left.wheel_motor,
        #     'rear_left_axle_joint': self.back_left.axle_motor,
        #     'rear_right_wheel_joint': self.back_right.wheel_motor,
        #     'rear_right_axle_joint': self.back_right.axle_motor,
        # }
        self.module_lookup = \
        {
            'front_left_axle_joint': self.front_left,
            'front_right_axle_joint': self.front_right,
            'rear_left_axle_joint': self.back_left,
            'rear_right_axle_joint': self.back_right,

        }

    def getEncoderInfo(self):
        output = \
        {
            'front_left': self.front_left.getEncoderInfo(),
            'front_right': self.front_right.getEncoderInfo(),
            'back_left': self.back_left.getEncoderInfo(),
            'back_right': self.back_right.getEncoderInfo()
        }
        return output

    def setTestVelocity(self, test_velocity, test_velocity2):
        scaled_vel = self.convertToTicks(test_velocity)
        scaled_vel2 = self.convertToTicks(test_velocity2)
        # self.test_motor.set(ctre.TalonFXControlMode.PercentOutput, test_velocity)
        # self.test_motor_2.set(ctre.TalonFXControlMode.PercentOutput, test_velocity2)
        # self.test.setVelocity(test_velocity, test_velocity2)

    def stop(self):
        self.front_left.stop()
        self.front_right.stop()
        self.back_left.stop()
        self.back_right.stop()
        # self.test_motor.set(ctre.TalonFXControlMode.PercentOutput, 0)
        # self.test_motor_2.set(ctre.TalonFXControlMode.PercentOutput, 0)


    def setDynamicVelocities(self, commands):
        if commands:
            for command in commands:
                module_axle_name = command['axle_joint']['name']
                module = self.module_lookup[module_axle_name]
                module.setVelocities(command['wheel_joint']['velocity'], command['axle_joint']['velocity'])

    def getTestEncoderInfo(self):
        return self.test_motor.getSensorCollection().getIntegratedSensorPosition(), self.test_motor.getSensorCollection().getIntegratedSensorVelocity()

