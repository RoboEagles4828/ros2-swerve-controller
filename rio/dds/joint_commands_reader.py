import rticonnextdds_connector as rti

class JointCommandsReader:
    
    def __init__(self):
        self.connector = rti.Connector(config_name="ROS2_PARTICIPANT_LIB::joint_commands",
        url="/home/lvuser/py/ROS_RTI.xml")
        self.input = self.connector.get_input("isaac_joint_commands_subscriber::joint_commands_reader")

        # print("Waiting for publications...")
        # self.output.wait_for_publications()

    def readData(self):
        #TODO: fix data format
        self.input.wait()
        self.input.take()
        for sample in self.input.samples.valid_data_iter:
            self.header = sample['header']
            self.velocity = sample['velocity']
            self.position = sample['position']
            self.effort = sample['effort']

        return {'header': self.header, 'velocity': self.velocity, 'position': self.position, 'effort': self.effort}