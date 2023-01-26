import wpilib
import dds.joystick_writer as joystick
import dds.joint_commands_reader as joint_cmds
import dds.encoder_info_writer as encoder_info
import hardware_interface.drivetrain as dt
import threading as mp
import time
import rticonnextdds_connector as rti
import os, inspect



#time.sleep() takes care of thread sleep yielding
class TestRobot(wpilib.TimedRobot):

    def robotInit(self) -> None:
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
        self._lock = mp.Lock()
        self._stop_threads = False

    def joystick_thread_ptr(self, controller):
        curr_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
        rel_path = "dds/xml/ROS_RTI.xml"
        xml_path = os.path.join(curr_path, rel_path)
        config_name="ROS2_PARTICIPANT_LIB::joystick"
        with rti.open_connector(config_name=config_name, url=xml_path) as connector:
            joystick_writer = joystick.JoyStickWriter(connector)
            while True:
                if self._stop_threads is True:
                    # joystick_writer.closeConnector()
                    break
                
                axes = [controller.getLeftX(), controller.getLeftY(), controller.getRightX(), controller.getRightY()]
                buttons = [int(controller.getAButton()), int(controller.getBButton()), int(controller.getXButton()), int(controller.getYButton())]

                joystick_writer.sendData(axes, buttons)
                time.sleep(20/1000) #20ms teleopPeriodic loop time
        
    def joint_commands_thread_ptr(self, drivetrain: dt.DriveTrain):
        curr_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
        rel_path = "dds/xml/ROS_RTI.xml"
        xml_path = os.path.join(curr_path, rel_path)
        config_name="ROS2_PARTICIPANT_LIB::joint_commands"
        with rti.open_connector(config_name=config_name, url=xml_path) as connector:
            joint_commands_reader = joint_cmds.JointCommandsReader(connector)
            while True:
                #TODO: fix the data format
                data = joint_commands_reader.readData()

                if self._stop_threads is True:
                    # joint_commands_reader.closeConnector()
                    break
                
                while data is None:
                    data = joint_commands_reader.readData()

                if self._stop_threads is True:
                    # joint_commands_reader.closeConnector()   
                    break

                if data:
                    if data == 'HALT':
                        print('RAMPING MOTORS DOWN')
                        with self._lock:
                            drivetrain.stop()
                        continue
                    with self._lock:
                        # drivetrain.setTestVelocity(run_wheel_velocities[0], turn_wheel_velocities[0])
                        drivetrain.setDynamicVelocities(data)

        print('CLEANING UP')

    def encoder_info_thread_ptr(self, drivetrain):
        curr_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
        rel_path = "dds/xml/ROS_RTI.xml"
        xml_path = os.path.join(curr_path, rel_path)
        config_name="ROS2_PARTICIPANT_LIB::encoder_info"
        with rti.open_connector(config_name=config_name, url=xml_path) as connector:
            encoder_info_writer = encoder_info.EncoderInfoWriter(connector)
            while True:
                info = None
                if self._stop_threads is True:
                    # encoder_info_writer.closeConnector()
                    break
                
                with self._lock:
                    info = drivetrain.getEncoderInfo()

                encoder_info_writer.sendData(info)
                time.sleep(20/1000) #20ms teleopPeriodic loop time

    # def pingWatchdog(self):
    #     watchdog = wpilib.Watchdog(20/1000)
    #     watchdog.reset()


    def teleopInit(self) -> None:
        print("Initializing Threads")
        # self.joystick_thread = mp.Thread(target=self.joystick_thread_ptr, name='joystick', args=(self.drivetrain.controller, ))
        # self.joint_commands_thread = mp.Thread(target=self.joint_commands_thread_ptr, name='joint_commands', args=(self.drivetrain, ))
        # self.encoder_info_thread = mp.Thread(target=self.encoder_info_thread_ptr, name='encoder_info')

        self.joystick_thread = {'name': 'joystick_thread', 'thread': None}
        self.joint_commands_thread = {'name': 'joint_commands_thread', 'thread': None}
        self.encoder_info_thread = {'name': 'encoder_info_thread', 'thread': None}

        self.threads = [self.joystick_thread, self.joint_commands_thread, self.encoder_info_thread]
        # self.threads = [self.joint_commands_thread]

    def start_thread(self, name):
        print(f'STARTING THREAD -- {name}')

        if 'joystick' in name:
            return mp.Thread(target=self.joystick_thread_ptr, name='joystick', args=(self.drivetrain.controller, ))
        elif 'joint' in name:
            return mp.Thread(target=self.joint_commands_thread_ptr, name='joint_commands', args=(self.drivetrain, ))
        elif 'encoder' in name:
            return mp.Thread(target=self.encoder_info_thread_ptr, name='encoder_info', args=(self.drivetrain, ))
        else:
            print(f'THREAD -- {name} -- NOT FOUND')

    def teleopPeriodic(self) -> None:
        # print(self.threads)
        for index in range(len(self.threads)):
            thread = self.threads[index]
            if thread['thread'] is None:
                thread['thread'] = self.start_thread(thread['name'])
                thread['thread'].start()
                self.threads[index] = thread
            else:
                if not thread['thread'].is_alive():
                    thread['thread'] = self.start_thread(thread['name'])
                    thread['thread'].start()
                    self.threads[index] = thread

    def stop_threads(self):
        self._stop_threads = True
        for thread in self.threads:
            print(f'Stopping Thread -- {thread["name"]}')
            thread['thread'].join()
        print('All Threads Stopped')


    def teleopExit(self) -> None:
        print("Exit")
        # self.pingWatchdog()
        self.stop_threads()
        self._stop_threads = False

    # def disabledInit(self) -> None:
    #     if mp.active_count() > 0:
    #         print("Killing all remaining threads")
    #         for thread in mp.enumerate():
    #             print(f'Killing {thread.name}')
    #             thread.join()
    #         print("All threads killed")

if __name__ == '__main__':
    # robot = TestRobot()
    # robot.robotInit()
    # robot.teleopInit()
    # while True:
    #     robot.teleopPeriodic()
    #     time.sleep(20/1000)
    wpilib.run(TestRobot)