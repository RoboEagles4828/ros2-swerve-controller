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

with rti.open_connector(
        config_name="ROS2_PARTICPANT_LIB::publisher",
        url="./ROS_RTI.xml") as connector:

    output = connector.get_output("jostick_data_publisher::jostick_data_writer")

    print("Waiting for subscriptions...")
    output.wait_for_subscriptions()

    print("Writing...")
    for i in range(1, 100):
        output.instance.set_number("x", i)
        output.instance.set_number("y", i*2)
        output.instance.set_number("shapesize", 30)
        output.instance.set_string("color", "BLUE")
        output.write()

        sleep(0.5) # Write at a rate of one sample every 0.5 seconds, for ex.

    print("Exiting...")
    output.wait() # Wait for all subscriptions to receive the data before exiting