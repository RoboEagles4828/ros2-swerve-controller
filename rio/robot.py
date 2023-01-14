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

    def joystick_thread_ptr(self, controller):
        joystick_writer = joystick.JoyStickWriter()
        while True:
            axes = [controller.getLeftX(), controller.getLeftY(), controller.getRightX(), controller.getRightY()]
            buttons = [int(controller.getAButton()), int(controller.getBButton()), int(controller.getXButton()), int(controller.getYButton())]
            joystick_writer.sendData(axes, buttons)
            time.sleep(20/1000) #20ms teleopPeriodic loop time
        
    def joint_commands_thread_ptr(self, drivetrain):
        joint_commands_reader = joint_cmds.JointCommandsReader()
        encoder_info_writer = encoder_info.EncoderInfoWriter()
        while True:
            #TODO: fix the data format
            data = joint_commands_reader.readData()
            if (data != None):
                print(data['velocity'])
                run_wheel_velocities = data['velocity'][:4]
                turn_wheel_velocities = data['velocity'][4:]
                print('run wheel vel: ' + str(run_wheel_velocities[0]))
                drivetrain.setTestVelocity(run_wheel_velocities[0])
            position = 1
            velocity = 1
            # encoder_info_writer.sendData(position, velocity)

    # def encoder_info_thread_ptr(self):
    #     while True:
    #         self.encoder_info_writer.sendData(self.position, self.velocity)
    #         time.sleep(20/1000) #20ms teleopPeriodic loop time


    def teleopInit(self) -> None:
        print("Initializing Threads")
        # self.joystick_thread = mp.Thread(target=self.joystick_thread_ptr, name='joystick', args=(self.drivetrain.controller, ))
        self.joint_commands_thread = mp.Thread(target=self.joint_commands_thread_ptr, name='joint_commands', args=(self.drivetrain, ))
        # self.encoder_info_thread = mp.Thread(target=self.encoder_info_thread_ptr, name='encoder_info')

        self.threads = [self.joint_commands_thread]

    def teleopPeriodic(self) -> None:
        if self.first:
            print("Starting Threads")
            for thread in self.threads:
                thread.start()
            self.first = False

    def teleopExit(self) -> None:
        for thread in self.threads:
            thread.join()
        super().teleopExit()


if __name__ == '__main__':
    # robot = TestRobot()
    # robot.robotInit()
    # robot.teleopInit()
    # while True:
    #     robot.teleopPeriodic()
    #     time.sleep(20/1000)
    wpilib.run(TestRobot)