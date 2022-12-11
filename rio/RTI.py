###############################################################################
# (c) 2005-2015 Copyright, Real-Time Innovations.  All rights reserved.       #
# No duplications, whole or partial, manual or electronic, may be made        #
# without express written permission.  Any such copies, or revisions thereof, #
# must display this notice unaltered.                                         #
# This code contains trade secrets of Real-Time Innovations, Inc.             #
###############################################################################

from time import sleep

# Updating the system path is not required if you have pip-installed
# rticonnextdds-connector

import rticonnextdds_connector as rti

class RTI_Node:

    def __init__(self):
        self.connector = rti.Conenctor(config_name="ROS2_PARTICPANT_LIB::publisher", url="./ROS_RTI.xml")
        self.output = self.connector.get_output("jostick_data_publisher::jostick_data_writer")

        print("Waiting for subscriptions...")
        self.output.wait_for_subscriptions()
    
    def sendData(self, axes, buttons):
            self.output.instance.set_dictionary({"axes": axes, "buttons": buttons})
            self.output.write()