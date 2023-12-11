import wpilib
import commands2
from RobotContainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    # robot
    def robotInit(self) -> None:
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.container = RobotContainer()
        self.container.swerve_subsystem().zeroHeading()

    def robotPeriodic(self) -> None:
        commands2.CommandScheduler.getInstance().run()

    # autonomus
    def autonomousInit(self) -> None:
        if self.container.get_autonomous_command() is  not None:
            self.container.get_autonomous_command().execute()
        pass

    def autonomousPeriodic(self) -> None:
        pass
    # teleoperated
    def teleopInit(self) -> None:
        if self.container.get_autonomous_command() is not None:
            self.container.get_autonomous_command().end()
        pass

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
