import rticonnextdds_connector as rti
import inspect
import os

class EncoderInfoWriter:

    def __init__(self):
        curr_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
        rel_path = "xml/ROS_RTI.xml"
        xml_path = os.path.join(curr_path, rel_path)
        print("XML PATH: ", xml_path)
        self.connector = rti.Connector(config_name="ROS2_PARTICIPANT_LIB::encoder_info", url=xml_path)
        self.output = self.connector.get_output("encoder_info_publisher::encoder_info_writer")

        # print("Waiting for subscriptions...")
        # self.output.wait_for_subscriptions()
    
    def sendData(self, positions, velocities):
        self.output.instance.set_dictionary({"position": positions, "velocity": velocities})
        self.output.write()