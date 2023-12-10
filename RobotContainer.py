from commands2 import Command
from Constants import Constants
from Subsytem.SwerveSubsystem import SwerveSubsystem
import commands2
import commands2.cmd
import commands2.button
from Constants import Constants
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.controller import PIDController, ProfiledPIDController
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

        self.driverController = commands2.button.CommandPS4Controller(
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
        self.driverController.circle().onTrue(
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
        config = TrajectoryConfig(
            Constants.Swerve.maxSpeed, Constants.Swerve.maxSpeed
        ).setKinematics(Constants.Swerve.oldSwerveKinematics)

        example_trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(0, 0, Rotation2d(0)),
            [Translation2d(1, 1), Translation2d(2, -1)],
            Pose2d(3, 0, Rotation2d(0)),
            config,
        )

        theta_controller = ProfiledPIDController(
            Constants.Swerve.thetaKP, 0, 0, Constants.Swerve.kThetaControllerConstraints
        )
        theta_controller.enableContinuousInput(-180, 180)

        swerve_controller_command = Swerve4ControllerCommand(
            example_trajectory,
            self.swerveSubsystem.getPose(),
            Constants.Swerve.oldSwerveKinematics,
            PIDController(Constants.Swerve.kPXController, 0, 0),
            PIDController(Constants.Swerve.kPYController, 0, 0),
            theta_controller,
            self.swerveSubsystem.setModuleStates,
            self.swerveSubsystem,
        )
        command_group = SequentialCommandGroup()
        command_group.addCommands(
            InstantCommand(
                lambda: self.swerveSubsystem.resetOdometry(
                    example_trajectory.getInitialPose()
                )
            )
        )
        command_group.addCommands(swerve_controller_command)

        return command_group


if __name__ == "__main__":
    # Instantiate RobotContainer and use its methods as needed
    robot_container = RobotContainer()
    auto_command = robot_container.get_autonomous_command()
    # ... Rest of your robot code ...
