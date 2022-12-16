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
        axes = [self.controller.getLeftX(), self.controller.getLeftY(), self.controller.getRightX(), self.controller.getRightY()]
        buttons = [int(self.controller.getAButton()), int(self.controller.getBButton()), int(self.controller.getXButton()), int(self.controller.getYButton())]
        # axes = [1, 2, 3, 4]
        # buttons = [4, 3, 2, 1]
        # buttons = [int(i) for i in buttons]
        self.writer.sendData(axes, buttons)
        print(f"Setting data: axes: {axes} buttons: {buttons}")
        print(int(self.controller.getAButton()))

if __name__ == '__main__':
    wpilib.run(TestRobot)