import rticonnextdds_connector as rti

class EncoderInfoWriter:

    def __init__(self):
        self.connector = rti.Connector(config_name="ROS2_PARTICIPANT_LIB::encoder_info",
        url="/home/lvuser/py/ROS_RTI.xml")
        self.output = self.connector.get_output("encoder_info_publisher::encoder_info_writer")

        # print("Waiting for subscriptions...")
        # self.output.wait_for_subscriptions()
    
    def sendData(self, positions, velocities):
        self.output.instance.set_dictionary({"position": positions, "velocity": velocities})
        self.output.write()