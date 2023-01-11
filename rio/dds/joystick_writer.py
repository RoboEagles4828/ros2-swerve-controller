import rticonnextdds_connector as rti

class JoyStickWriter:

    def __init__(self):
        self.connector = rti.Connector(config_name="ROS2_PARTICIPANT_LIB::joystick",
        url="/home/saranga/FRC/RTI_DDS/orin_to_rio/2023RobotROS/rio/dds/xml/ROS_RTI.xml")
        self.output = self.connector.get_output("joystick_data_publisher::joystick_data_writer")

        # print("Waiting for subscriptions...")
        # self.output.wait_for_subscriptions()
    
    def sendData(self, axes, buttons):
        self.output.instance.set_dictionary({"axes": axes, "buttons": buttons})
        self.output.write()