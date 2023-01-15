import wpilib
import dds.joystick_writer as joystick
import dds.joint_commands_reader as joint_cmds
import dds.encoder_info_writer as encoder_info
import hardware_interface.drivetrain as dt
import multiprocessing as mp
import time



#time.sleep() takes care of thread sleep yielding
class TestRobot(wpilib.TimedRobot):

    def robotInit(self) -> None:
        self.drivetrain = dt.DriveTrain()
        self.controller = wpilib.XboxController(0)
        self.joystick_writer = joystick.JoyStickWriter()
        

    def teleopInit(self) -> None:
        print('Init')

    def teleopPeriodic(self) -> None:
        # self.axes = [self.controller.getLeftX(), self.controller.getLeftY(), self.controller.getRightX(), self.controller.getRightY()]
        # self.buttons = [int(self.controller.getAButton()), int(self.controller.getBButton()), int(self.controller.getXButton()), int(self.controller.getYButton())]
        # self.axes = [0]*4
        # self.buttons = [0]*4
        # self.joystick_writer.sendData(self.axes, self.buttons)
        self.drivetrain.setTestVelocity(50)

if __name__ == '__main__':
    # robot = TestRobot()
    # robot.robotInit()
    # robot.teleopInit()
    # while True:
    #     robot.teleopPeriodic()
    wpilib.run(TestRobot)