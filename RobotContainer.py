from commands2 import Command
from Constants import OIConstants
from Subsytem import SwerveSubsystem
import commands2
import commands2.cmd
import commands2.button

class RobotContainer:

    def __init__(self):
        self.swerveSubsystem = SwerveSubsystem.SwerveSubsystem()

        self.driverController = commands2.button.CommandPS4Controller(OIConstants.kDriverControllerPort)
        commands2.CommandScheduler.setDefaultCommand(
            commands2.cmd.run(
                lambda: self.swerveSubsystem.drive(
                    self.driverController.getLeftX(),
                    self.driverController.getLeftY(),
                    self.driverController.getRightX(),
                    self.driverController.getCrossButton()
                )
            )
        )
        self.configure_button_bindings()

    def configure_button_bindings(self):
        pass
    def get_autonomous_command(self) -> Command:
        pass

if __name__ == "__main__":
    # Instantiate RobotContainer and use its methods as needed
    robot_container = RobotContainer()
    auto_command = robot_container.get_autonomous_command()
    # ... Rest of your robot code ...
