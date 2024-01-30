from typing import Tuple
from commands2 import (
    Command,
    Subsystem,
    SwerveControllerCommand,
    SequentialCommandGroup,
)
from Constants import OIConstants
from Subsytem.SwerveSubsystem import SwerveSubsystem
import commands2
import commands2.cmd
import commands2.button
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.geometry import Pose2d
from wpimath.controller import (
    PIDController,
    ProfiledPIDControllerRadians,
    HolonomicDriveController,
)
from Constants import DriveConstants, AutoConstants
import math


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
                    self.driverController.getLeftY(),
                    self.driverController.getLeftX(),
                    -self.driverController.getRightX(),
                )
            )
        )
        self.configure_button_bindings()

    def configure_button_bindings(self):
        self.driverController.b().onTrue(
            commands2.cmd.runOnce(lambda: self.swerveSubsystem.zeroHeading())
        )
        pass

    def get_autonomous_command(self) -> Command:
        config: TrajectoryConfig = TrajectoryConfig(
            DriveConstants.swerve_max_speed,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        )
        config.setKinematics(DriveConstants.kDriveKinematics)

        pose2d_list: list[Pose2d] = list()
        pose2d_list.append(Pose2d(0, 0, 0))
        pose2d_list.append(Pose2d(0.5, 0, 0))
        pose2d_list.append(Pose2d(0.5, 0.5, 0))
        pose2d_list.append(Pose2d(0, 0, 0))
        trajectory = TrajectoryGenerator.generateTrajectory(
            pose2d_list,
            config,
        )

        theta_controller = ProfiledPIDControllerRadians(
            0.1, 0, 0, AutoConstants.kThetaControllerConstraints
        )
        theta_controller.enableContinuousInput(-math.pi, math.pi)

        xController = PIDController(AutoConstants.kPXController, 0, 0)
        yController = PIDController(AutoConstants.kPYController, 0, 0)

        holController: HolonomicDriveController = HolonomicDriveController(
            xController, yController, theta_controller
        )
        req: Tuple[Subsystem] = (self.swerveSubsystem,)

        swerve_controller_command = SwerveControllerCommand(
            trajectory,
            self.swerveSubsystem.getPose,
            DriveConstants.kDriveKinematics,
            holController,
            self.swerveSubsystem.setModuleStates,
            requirements=req,
        )
        command_group = SequentialCommandGroup()
        command_group.addCommands(
            commands2.InstantCommand(
                lambda: self.swerveSubsystem.resetOdometry(trajectory.initialPose())
            )
        )
        command_group.addCommands(swerve_controller_command)

        return command_group
