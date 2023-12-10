
import wpilib
import commands2
from RobotContainer import RobotContainer

class MyRobot(commands2.TimedCommandRobot):
    #robot
    def robotInit(self) -> None:
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.container = RobotContainer()
        self.container.swerve_subsystem().zeroHeading()
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
        #self.container.get_swerve().print_CANCoder_values()
        #self.container.get_swerve().print_NEO_encoder_values()
        self.container.get_swerve().print_gyro_angle()
        #self.container.print_joystick()
        pass
    def testInit(self) -> None:
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)