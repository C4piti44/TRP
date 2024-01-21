import wpilib
import commands2
from RobotContainer import RobotContainer

class MyRobot(commands2.TimedCommandRobot):
    # robot
    def robotInit(self) -> None:#this function will run once on enabling the robot
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.container = RobotContainer()
        self.auto_command: commands2.Command = self.container.get_autonomous_command()

    def robotPeriodic(self) -> None:#this function will run until the robot is desaibled or turned off
        commands2.CommandScheduler.getInstance().run()

    # autonomus
    def autonomousInit(self) -> None:#this funtion will run once in the autonomous mode
        if self.auto_command is not None:
            self.auto_command.schedule()
        pass

    def autonomousPeriodic(self) -> None:#this function will run from the start of the auto mode until its end
        pass

    # teleoperated
    def teleopInit(self) -> None:#this function will run once in the teleop mode
        if self.auto_command is not None:
            self.auto_command.cancel()
        pass

    def teleopPeriodic(self) -> None:#this function will run from the start of the teleop mode until its end
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)
