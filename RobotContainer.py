from commands2 import Command
from Constants import Constants
from Subsytem.SwerveSubsystem import SwerveSubsystem
import commands2
import commands2.cmd
import commands2.button
from Constants import Constants
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from commands2 import (
    InstantCommand,
    SequentialCommandGroup,
    Swerve4ControllerCommand,
    CommandScheduler,
)
import math


class RobotContainer:
    def __init__(self):
        self.swerveSubsystem = SwerveSubsystem()

        self.driverController = commands2.button.CommandXboxController(
            Constants.OIConstants.kDriverControllerPort
        )
        self.swerveSubsystem.setDefaultCommand(
            self.swerveSubsystem.run(
                lambda: SwerveSubsystem.drive(
                    self.swerveSubsystem,
                    self.driverController.getLeftX(),
                    self.driverController.getLeftY(),
                    self.driverController.getRightX(),
                )
            )
        )
        self.driverController.B().onTrue(
            commands2.cmd.runOnce(lambda: self.swerveSubsystem.zeroHeading())
        )
        self.configure_button_bindings()

    def configure_button_bindings(self):
        pass

    def get_swerve(self) -> SwerveSubsystem:
        return self.swerveSubsystem

    def print_joystick(self) -> None:
        print(f"Left Y: {str(self.driverController.getLeftY())}")
        print(f"Left X: {str(self.driverController.getLeftX())}")
        print(f"Right X: {str(self.driverController.getRightX())}")

    def swerve_subsystem(self):
        return self.swerveSubsystem

    def get_autonomous_command(self) -> SequentialCommandGroup:
        return None
        config = TrajectoryConfig(
            Constants.Swerve.maxSpeed, Constants.Swerve.maxSpeed
        ).setKinematics(Constants.Swerve.oldSwerveKinematics)

        translation_list = list[Translation2d(1, 1), Translation2d(2, 2)]

        trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(0, 0, Rotation2d(0)),
            translation_list,
            Pose2d(3, 0, Rotation2d(0)),
            config,
        )

        theta_controller = ProfiledPIDControllerRadians(
            Constants.Swerve.thetaKP, 0, 0, Constants.Swerve.kThetaControllerConstraints
        )
        theta_controller.enableContinuousInput(-math.pi, math.pi)

        swerve_controller_command = Swerve4ControllerCommand(
            trajectory,
            self.swerveSubsystem.getPose(),
            Constants.Swerve.oldSwerveKinematics,
            PIDController(Constants.Swerve.kPXController, 0, 0),
            PIDController(Constants.Swerve.kPYController, 0, 0),
            theta_controller,
            self.swerveSubsystem.setModuleStates(),
            self.swerveSubsystem,
        )
        command_group = SequentialCommandGroup()
        command_group.addCommands(swerve_controller_command)

        return command_group


if __name__ == "__main__":
    # Instantiate RobotContainer and use its methods as needed
    robot_container = RobotContainer()
    auto_command = robot_container.get_autonomous_command()
    # ... Rest of your robot code ...
