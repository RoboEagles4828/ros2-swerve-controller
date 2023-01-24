import rticonnextdds_connector as rti
import os, inspect

class JoyStickWriter:

    def __init__(self, connector):
        # self.connector = rti.Connector(config_name="ROS2_PARTICIPANT_LIB::joystick", url=xml_path)
        self.output = connector.get_output("joystick_data_publisher::joystick_data_writer")
        # print("Waiting for subscriptions...")
        # self.output.wait_for_subscriptions()
    
    def sendData(self, axes, buttons):
        if axes and buttons:
            self.output.instance.set_dictionary({"axes": axes, "buttons": buttons})
            self.output.write()

    # def closeConnector(self):
    #     print("CLOSING JOYSTICK CONNECTOR")
    #     self.connector.close()