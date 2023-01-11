import wpilib
import dds.joystick_writer as joystick
import dds.joint_commands_reader as joint_cmds
import dds.encoder_info_writer as encoder_info
import hardware_interface.drivetrain as dt
import threading as mp
import time



#time.sleep() takes care of thread sleep yielding
class TestRobot(wpilib.TimedRobot):

    def robotInit(self) -> None:
        self.joystick_writer = joystick.JoyStickWriter()
        self.joint_commands_reader = joint_cmds.JointCommandsReader()
        self.encoder_info_writer = encoder_info.EncoderInfoWriter()
        self.drivetrain = dt.DriveTrain()
        self.controller = self.drivetrain.controller
        self.axes = []
        self.buttons = []
        self.position = []
        self.velocity = []
        self.run_wheel_velocities = []
        self.turn_wheel_velocities = []
        self.threads = []
        self.first = True

    def joystick_thread_ptr(self):
        while True:
            self.joystick_writer.sendData(self.axes, self.buttons)
            time.sleep(20/1000) #20ms teleopPeriodic loop time
        
    def joint_commands_thread_ptr(self):
        while True:
            #TODO: fix the data format
            data = self.joint_commands_reader.readData()
            self.run_wheel_velocities = data['velocity'][:4]
            self.turn_wheel_velocities = data['velocity'][4:]
            time.sleep(20/1000) #20ms teleopPeriodic loop time

    def encoder_info_thread_ptr(self):
        while True:
            self.encoder_info_writer.sendData(self.position, self.velocity)
            time.sleep(20/1000) #20ms teleopPeriodic loop time


    def teleopInit(self) -> None:
        print("Initializing Threads")
        self.joystick_thread = mp.Thread(target=self.joystick_thread_ptr, name='joystick')
        self.joint_commands_thread = mp.Thread(target=self.joint_commands_thread_ptr, name='joint_commands')
        self.encoder_info_thread = mp.Thread(target=self.encoder_info_thread_ptr, name='encoder_info')

        self.threads = [self.joystick_thread, self.joint_commands_thread, self.encoder_info_thread]

    def teleopPeriodic(self) -> None:
        if self.first:
            print("Starting Threads")
            for thread in self.threads:
                thread.start()
            self.first = False
        self.axes = [self.controller.getLeftX(), self.controller.getLeftY(), self.controller.getRightX(), self.controller.getRightY()]
        self.buttons = [int(self.controller.getAButton()), int(self.controller.getBButton()), int(self.controller.getXButton()), int(self.controller.getYButton())]
        # self.axes = [0]*4
        # self.buttons = [0]*4
        self.position = self.drivetrain.getEncoderInfo()['position']
        self.velocity = self.drivetrain.getEncoderInfo()['velocity']
        self.joystick_writer.sendData(self.axes, self.buttons)
        self.encoder_info_writer.sendData(self.position, self.velocity)
        data = self.joint_commands_reader.readData()
        self.run_wheel_velocities = data['velocity'][:4]
        self.turn_wheel_velocities = data['velocity'][4:]
        print(self.run_wheel_velocities)
        print(self.turn_wheel_velocities)
        # self.drivetrain.setVelocities(self.run_wheel_velocities, self.turn_wheel_velocities)

    # def teleopExit(self) -> None:
    #     for thread in self.threads:
    #         thread.terminate()
    #     super().teleopExit()


if __name__ == '__main__':
    lksjdafs = TestRobot()
    lksjdafs.robotInit()
    lksjdafs.teleopInit()
    while True:
        lksjdafs.teleopPeriodic()