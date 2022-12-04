###############################################################################
# (c) 2005-2015 Copyright, Real-Time Innovations.  All rights reserved.       #
# No duplications, whole or partial, manual or electronic, may be made        #
# without express written permission.  Any such copies, or revisions thereof, #
# must display this notice unaltered.                                         #
# This code contains trade secrets of Real-Time Innovations, Inc.             #
###############################################################################

from __future__ import print_function

# Updating the system path is not required if you have pip-installed
# rticonnextdds-connector

import rticonnextdds_connector as rti

with rti.open_connector(
        config_name="ROS2_PARTICIPANT_LIB::subscriber",
        url="./ROS_RTI.xml") as connector:

    input = connector.get_input("rio_data_subscriber::rio_data_reader")

    print("Waiting for publications...")
    input.wait_for_publications() # wait for at least one matching publication

    print("Waiting for data...")
    for i in range(1, 500):
        input.wait() # wait for data on this input
        input.take()
        for sample in input.samples.valid_data_iter:
            # You can get all the fields in a get_dictionary()
            data = sample.get_dictionary()
            axes = data['axes']
            buttons = data['buttons']
            
            print("Axes: " + str(axes) + "\nButtons: " + str(buttons))
