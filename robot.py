import wpilib
import commands2
from RobotContainer import RobotContainer
from cscore import CameraServer as CS
class MyRobot(commands2.TimedCommandRobot):
    # robot
    def robotInit(self) -> None:
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.container = RobotContainer()
        self.auto_command = self.container.get_autonomous_command()
        self.container.swerveSubsystem.zeroHeading()
        self.camera0 = CS.startAutomaticCapture(0)

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

    def teleopPeriodic(self) -> None:
        self.container.angulator.setAngle()

    def testInit(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
