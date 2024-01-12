from commands2 import Command
from Constants import OIConstants
from Subsytem.SwerveSubsystem import SwerveSubsystem
import commands2
import commands2.cmd
import commands2.button
from wpimath import filter


class RobotContainer:
    def __init__(self):
        self.swerveSubsystem = SwerveSubsystem()

        self.driverController = commands2.button.CommandXboxController(
            OIConstants.kDriverControllerPort
        )
        self.swerveSubsystem.setDefaultCommand(
            self.swerveSubsystem.run(
                lambda: SwerveSubsystem.drive(
                    self.swerveSubsystem,
                    -self.driverController.getLeftY(),
                    -self.driverController.getLeftX(),
                    -self.driverController.getRightX(),
                )
            )
        )
        self.configure_button_bindings()

    def configure_button_bindings(self):
        self.driverController.B().onTrue(
            commands2.cmd.runOnce(
                lambda: self.swerveSubsystem.zeroHeading()
            )
        )

    def get_autonomous_command(self) -> Command:
        pass


if __name__ == "__main__":
    # Instantiate RobotContainer and use its methods as needed
    robot_container = RobotContainer()
    auto_command = robot_container.get_autonomous_command()
    # ... Rest of your robot code ...
