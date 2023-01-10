import wpilib
import rio.dds.joystick_writer as joystick
import rio.dds.joint_commands_reader as joint_cmds
import rio.dds.encoder_info_writer as encoder_info
import drivetrain as dt
import multiprocessing as mp
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
            self.run_wheel_velocities = data['velocity']
            self.turn_wheel_velocities = data['position']
            time.sleep(20/1000) #20ms teleopPeriodic loop time

    def encoder_info_thread_ptr(self):
        while True:
            self.encoder_info_writer.sendData(self.position, self.velocity)
            time.sleep(20/1000) #20ms teleopPeriodic loop time


    def teleopInit(self) -> None:
        print("Initializing Threads")
        self.joystick_thread = mp.Process(target=self.joystick_thread_ptr, name='joystick')
        self.joint_commands_thread = mp.Process(target=self.joint_commands_thread_ptr, name='joint_commands')
        self.encoder_info_thread = mp.Process(target=self.encoder_info_thread_ptr, name='encoder_info')

        self.threads = [self.joystick_thread, self.joint_commands_thread, self.encoder_info_thread]

    def teleopPeriodic(self) -> None:
        if self.first:
            print("Starting Threads")
            for thread in self.threads:
                thread.start()
            self.first = False
        self.axes = [self.controller.getLeftX(), self.controller.getLeftY(), self.controller.getRightX(), self.controller.getRightY()]
        self.buttons = [int(self.controller.getAButton()), int(self.controller.getBButton()), int(self.controller.getXButton()), int(self.controller.getYButton())]
        self.position = self.drivetrain.getEncoderInfo()['position']
        self.velocity = self.drivetrain.getEncoderInfo()['velocity']
        self.drivetrain.setVelocities(self.run_wheel_velocities, self.turn_wheel_velocities)

    def teleopExit(self) -> None:
        for thread in self.threads:
            thread.terminate()
        super().teleopExit()


if __name__ == '__main__':
    wpilib.run(TestRobot)