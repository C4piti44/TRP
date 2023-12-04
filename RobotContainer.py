from commands2 import Command
from Constants import OIConstants
from Subsytem.SwerveSubsystem import SwerveSubsystem
import commands2
import commands2.cmd
import commands2.button
import math

class RobotContainer:

    def __init__(self):
        self.swerveSubsystem = SwerveSubsystem()
        
        self.driverController = commands2.button.CommandPS4Controller(OIConstants.kDriverControllerPort)
        self.swerveSubsystem.setDefaultCommand(
            self.swerveSubsystem.run(
                lambda: SwerveSubsystem.drive(
                    self.swerveSubsystem,
                    self.driverController.getLeftX(),
                    self.driverController.getLeftY(),
                    self.driverController.getRightX()
                )
            )
        )

        self.configure_button_bindings()

    def configure_button_bindings(self):
        pass
    def get_autonomous_command(self) -> Command:
        pass
    def get_swerve(self) -> SwerveSubsystem:
        return self.swerveSubsystem
    def print_joystick(self) -> None:
        print(f"Left Y: {str(self.driverController.getLeftY())}")
        print(f"Left X: {str(self.driverController.getLeftX())}")
        print(f"Right X: {str(self.driverController.getRightX())}")

if __name__ == "__main__":
    # Instantiate RobotContainer and use its methods as needed
    robot_container = RobotContainer()
    auto_command = robot_container.get_autonomous_command()
    # ... Rest of your robot code ...
