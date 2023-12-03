from commands2 import Command
from Constants import OIConstants
from Subsytem import SwerveSubsystem
import commands2
import commands2.cmd
import commands2.button
import math

class RobotContainer:

    def __init__(self):
        self.swerveSubsystem = SwerveSubsystem.SwerveSubsystem()

        self.driverController = commands2.button.CommandPS4Controller(OIConstants.kDriverControllerPort)
        self.driverController.cross().onFalse(
            commands2.cmd.run(
                lambda: self.swerveSubsystem.drive(
                    self.driverController.getLeftX(),
                    self.driverController.getLeftY(),
                    self.driverController.getRightX(),
                    True,
                )
            )
        )

        self.configure_button_bindings()

    def configure_button_bindings(self):
        pass
    def get_autonomous_command(self) -> Command:
        pass
    def get_gyro(self) -> float:
        return self.swerveSubsystem.get_angle()
    def print_CANCoder(self) -> None:
        self.swerveSubsystem.print_CANCoder_values()
    def print_NEO_encoder(self) -> None:
        self.swerveSubsystem.print_NEO_encoder_values()
if __name__ == "__main__":
    # Instantiate RobotContainer and use its methods as needed
    robot_container = RobotContainer()
    auto_command = robot_container.get_autonomous_command()
    # ... Rest of your robot code ...
