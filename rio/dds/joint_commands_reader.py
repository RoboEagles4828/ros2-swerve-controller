import rticonnextdds_connector as rti
import os, inspect
import time

class JointCommandsReader:
    
    def __init__(self, connector):
        # self.connector = rti.Connector(config_name="ROS2_PARTICIPANT_LIB::joint_commands", url=xml_path)
        self.input = connector.get_input("isaac_joint_commands_subscriber::joint_commands_reader")
        self.header = list()
        self.velocity = list()
        self.last = time.time()

    def readData(self):
        # try:
        #     print("Waiting for publications...")
        #     self.input.wait_for_publications(20)
        # except:
        #     return None
        #TODO: fix data format
        cmds = list()
        header = list()
        velocity = list()
        names = list()
        module_cmds = list()
        if time.time() - self.last > 1:
            self.last = time.time()
            return 'HALT'
        try:
            self.input.wait(20)
        except:
            return None

        self.input.take()
        for sample in self.input.samples:
            if sample.valid_data:
                header = sample.get_dictionary()["header"]
                velocity = sample.get_dictionary()["velocity"]
                names = sample.get_dictionary()["name"]
                self.last = time.time()

        if names:
            for i, name in enumerate(names):
                cmds.append({'name': name, 'velocity': velocity[i]})
            
            for axle_cmd in filter(lambda x: 'axle' in x['name'], cmds):
                wheel_joint_name = axle_cmd['name'].replace('axle', 'wheel')
                wheel_cmd = next(filter(lambda x: x['name'] == wheel_joint_name, cmds))
                module_cmd = {'wheel_joint': wheel_cmd, 'axle_joint': axle_cmd}       
                module_cmds.append(module_cmd)  

            # print(cmds)
        return module_cmds

    # def closeConnector(self):
    #     print("CLOSING JOINT COMMANDS CONNECTOR")
    #     self.connector.close()