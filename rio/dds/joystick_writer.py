import rticonnextdds_connector as rti
import os, inspect

class JoyStickWriter:

    def __init__(self):
        curr_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
        rel_path = "xml/ROS_RTI.xml"
        xml_path = os.path.join(curr_path, rel_path)
        self.connector = rti.Connector(config_name="ROS2_PARTICIPANT_LIB::joystick", url=xml_path)
        self.output = self.connector.get_output("joystick_data_publisher::joystick_data_writer")

        # print("Waiting for subscriptions...")
        # self.output.wait_for_subscriptions()
    
    def sendData(self, axes, buttons):
        self.output.instance.set_dictionary({"axes": axes, "buttons": buttons})
        self.output.write()