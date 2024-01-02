import wpilib
import commands2
from RobotContainer import RobotContainer
from rev import CANSparkMax, CANSparkMaxLowLevel


class MyRobot(commands2.TimedCommandRobot):
    # robot
    def robotInit(self) -> None:
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.container = RobotContainer()
        self.auto_command: commands2.Command = self.container.get_autonomous_command()

    def robotPeriodic(self) -> None:
        commands2.CommandScheduler.getInstance().run()

    # autonomus
    def autonomousInit(self) -> None:
        if self.auto_command is not None:
            self.auto_command.schedule()
        pass

    def autonomousPeriodic(self) -> None:
        pass

    # teleoperated
    def teleopInit(self) -> None:
        if self.auto_command is not None:
            self.auto_command.cancel()
        pass

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
