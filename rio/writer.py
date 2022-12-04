###############################################################################
# (c) 2005-2015 Copyright, Real-Time Innovations.  All rights reserved.       #
# No duplications, whole or partial, manual or electronic, may be made        #
# without express written permission.  Any such copies, or revisions thereof, #
# must display this notice unaltered.                                         #
# This code contains trade secrets of Real-Time Innovations, Inc.             #
###############################################################################

from time import sleep
import wpilib_joystick

# Updating the system path is not required if you have pip-installed
# rticonnextdds-connector

import rticonnextdds_connector as rti

with rti.open_connector(
        config_name="ROS2_PARTICPANT_LIB::publisher",
        url="./ROS_RTI.xml") as connector:

    output = connector.get_output("jostick_data_publisher::jostick_data_writer")

    print("Waiting for subscriptions...")
    output.wait_for_subscriptions()

    print("Writing...")
    while (True):
        axes = wpilib_joystick.getAxes()
        buttons = wpilib_joystick.getButtons()
        output.instance.set_dictionary({"axes":axes, "buttons":buttons})
        output.write()

        sleep(0.5) # Write at a rate of one sample every 0.5 seconds, for ex.

    print("Exiting...")
    output.wait() # Wait for all subscriptions to receive the data before exiting