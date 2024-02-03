import wpilib
import commands2
from RobotContainer import RobotContainer
from ntcore import NetworkTableInstance
from rev import CANSparkMax


class MyRobot(commands2.TimedCommandRobot):
    # robot
    def robotInit(self) -> None:
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.container = RobotContainer()
        self.auto_command = self.container.get_autonomous_command()
        self.limelight = NetworkTableInstance.getDefault().getTable("limelight")
        self.motor1 = CANSparkMax(9, CANSparkMax.MotorType.kBrushless)
        self.motor2 = CANSparkMax(10, CANSparkMax.MotorType.kBrushless)

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
        self.motor1.set(1)
        self.motor2.set(-1)
        pass

    def testInit(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
