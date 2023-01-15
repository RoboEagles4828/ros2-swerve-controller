import rticonnextdds_connector as rti
import os, inspect

class JointCommandsReader:
    
    def __init__(self):
        curr_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
        rel_path = "xml/ROS_RTI.xml"
        xml_path = os.path.join(curr_path, rel_path)
        self.connector = rti.Connector(config_name="ROS2_PARTICIPANT_LIB::joint_commands", url=xml_path)
        self.input = self.connector.get_input("isaac_joint_commands_subscriber::joint_commands_reader")
        self.header = []
        self.velocity = []

        # print("Waiting for publications...")
        # self.output.wait_for_publications()

    def readData(self):
        #TODO: fix data format
        header = []
        velocity = []
        try:
            self.input.wait(20)
        except:
            return None

        self.input.take()
        for sample in self.input.samples:
            if sample.valid_data:
                header = sample.get_dictionary()["header"]
                velocity = sample.get_dictionary()["velocity"]
        return {'header': header, 'velocity': velocity}