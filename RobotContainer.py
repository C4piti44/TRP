from commands2 import Command
from Constants import OIConstants
from Subsytem.SwerveSubsystem import SwerveSubsystem
import commands2
import commands2.cmd
import commands2.button
import math

class RobotContainer:

    def __init__(self):
        self.swerveSubsystem = SwerveSubsystem
        
        commands2._impl.Command
        
        self.driverController = commands2.button.CommandPS4Controller(OIConstants.kDriverControllerPort)
        self.swerveSubsystem.setDefaultCommand(
            self.swerveSubsystem.run(self.swerveSubsystem, self.swerveSubsystem.drive(
                self.swerveSubsystem
                ,self.driverController.getLeftX()
                ,self.driverController.getLeftY()
                ,self.driverController.getRightX()
            ))
        )

        self.configure_button_bindings()

    def configure_button_bindings(self):
        pass
    def get_autonomous_command(self) -> Command:
        pass
    def get_gyro(self) -> float:
        return self.swerveSubsystem.get_angle()
    def get_swerve(self) -> SwerveSubsystem:
        return self.swerveSubsystem
    def get_controller(self) -> commands2.button.CommandPS4Controller:
        return self.driverController

if __name__ == "__main__":
    # Instantiate RobotContainer and use its methods as needed
    robot_container = RobotContainer()
    auto_command = robot_container.get_autonomous_command()
    # ... Rest of your robot code ...
