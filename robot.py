
import wpilib
import commands2
from RobotContainer import RobotContainer

class MyRobot(commands2.TimedCommandRobot):
    #robot
    def robotInit(self) -> None:
        self.container = RobotContainer()
    def robotPeriodic(self) -> None:
        commands2.CommandScheduler.getInstance().run()
    #autonomus
    def autonomousInit(self) -> None:
        pass
    def autonomousPeriodic(self) -> None:
        pass

    #teleoperated
    def teleopInit(self) -> None:
        pass
    def teleopPeriodic(self) -> None:
        print(self.container.get_gyro())
    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()

if __name__ == "__main__":
    wpilib.run(MyRobot)