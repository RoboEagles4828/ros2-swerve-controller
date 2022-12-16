import rticonnextdds_connector as rti
from time import sleep

with rti.open_connector(config_name="ROS2_PARTICIPANT_LIB::publisher",
        url="./ROS_RTI.xml") as connector:

    output = connector.get_output("joystick_data_publisher::joystick_data_writer")

    print("Waiting for subscriptions...")
    output.wait_for_subscriptions() 

    axes = [1, 2, 3, 4]
    buttons = [4, 3, 2, 1]
    while(True):
        output.instance.set_dictionary({"axes": axes, "buttons": buttons})
        output.write()
        sleep(1)
        print(f'sending\naxes: {axes}\nbuttons:{buttons}')
        axes = [n+1 for n in axes]
        buttons = [n+1 for n in buttons]