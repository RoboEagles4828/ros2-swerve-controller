import wpilib

class TestRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.port = 0
        self.controller = wpilib.XboxController(0)
        self.axes = []
        self.buttons = []

    def teleopInit(self) -> None:
        print("Initialized")

    def teleopPeriodic(self) -> None:
        self.axes = [self.controller.getLeftX(), self.controller.getLeftY(), self.controller.getRightX(), self.controller.getRightY()]
        self.buttons = [self.controller.getAButton(), self.controller.getBButton(), self.controller.getXButton(), self.controller.getYButton()]
        print("Setting data: " + self.axes + " " + self.buttons)

    def getAxes(self):
        return self.axes

    def getButtons(self):
        return self.buttons

def getAxes():
    return TestRobot.getAxes()

def getButtons():
    return TestRobot.getButtons()

if __name__ == '__main__':
    wpilib.run(TestRobot)