import wpilib
import RTI

class TestRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.port = 0
        self.controller = wpilib.XboxController(0)
        self.axes = []
        self.buttons = []
        self.writer = RTI.RTI_Node()

    def teleopInit(self) -> None:
        print("Initialized")

    def teleopPeriodic(self) -> None:
        self.axes = [self.controller.getLeftX(), self.controller.getLeftY(), self.controller.getRightX(), self.controller.getRightY()]
        self.buttons = [self.controller.getAButton(), self.controller.getBButton(), self.controller.getXButton(), self.controller.getYButton()]
        self.writer.sendData(self.axes, self.buttons)
        print("Setting data: " + self.axes + " " + self.buttons)

if __name__ == '__main__':
    wpilib.run(TestRobot)