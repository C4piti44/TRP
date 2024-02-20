from commands2 import Command, SequentialCommandGroup
from Constants import (
    OIConstants,
    DriveConstants,
    ModuleConstants,
    ConveyanceConstants,
    ShooterConstants,
    IntakeConstants,
    angulatorConstants,
)
from Subsytem.SwerveSubsystem import SwerveSubsystem
import commands2
import commands2.cmd
import commands2.button
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState
from pathplannerlib.auto import AutoBuilder
from Subsytem.Shooter import Shooter
from Subsytem.conveyance import Conveyance
from Subsytem.intake import Intake
from Subsytem.angulator import angulator
from Subsytem.limelight import limelight
from wpimath.geometry import Pose2d, Rotation2d
import math


class RobotContainer:
    def __init__(self):
        self.swerveSubsystem = SwerveSubsystem()
        self.shooter = Shooter()
        self.conveyance = Conveyance()
        self.intake = Intake()
        self.angulator = angulator()
        self.limelight = limelight()

        self.driverController = commands2.button.CommandXboxController(
            OIConstants.kDriverControllerPort
        )
        self.operatorController = commands2.button.CommandJoystick(
            OIConstants.kOperatorControllerPort
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
        # reset robots heading
        self.driverController.b().onTrue(
            commands2.cmd.runOnce(lambda: self.swerveSubsystem.zeroHeading())
        )

        # conveyance
        self.operatorController.button(7).onTrue(
            commands2.cmd.runOnce(
                lambda: self.conveyance.move(ConveyanceConstants.moveForwardPower)
            )
        )
        self.operatorController.button(7).onFalse(
            commands2.cmd.runOnce(lambda: self.conveyance.move(0))
        )

        self.operatorController.button(3).onTrue(
            commands2.cmd.runOnce(
                lambda: self.conveyance.move(ConveyanceConstants.moveBackwardsPower)
            )
        )
        self.operatorController.button(3).onFalse(
            commands2.cmd.runOnce(lambda: self.conveyance.move(0))
        )

        self.operatorController.button(1).onTrue(
            commands2.cmd.runOnce(
                lambda: self.conveyance.amp(
                    -ConveyanceConstants.moveForwardPower - 0.1,
                    ConveyanceConstants.moveBackwardsPower / 6,
                )
            )
        )
        self.operatorController.button(1).onFalse(
            commands2.cmd.runOnce(lambda: self.conveyance.move(0))
        )

        # shooter
        self.operatorController.button(4).onTrue(
            commands2.cmd.runOnce(
                lambda: self.shooter.shoot(-ShooterConstants.shootPower)
            )
        )
        self.operatorController.button(4).onFalse(
            commands2.cmd.runOnce(lambda: self.shooter.shoot(0))
        )

        # intake
        self.operatorController.button(10).onTrue(
            commands2.cmd.runOnce(lambda: self.intake.move(IntakeConstants.intakePower))
        )
        self.operatorController.button(10).onFalse(
            commands2.cmd.runOnce(lambda: self.intake.move(0))
        )

        # angulator
        self.operatorController.button(12).whileTrue(
            commands2.cmd.runOnce(
                lambda: self.angulator.setAngle(angulatorConstants.angle_of_amp)
            )
        )
        self.operatorController.button(12).whileFalse(
            commands2.cmd.runOnce(lambda: self.angulator.setAngle(0))
        )

        self.operatorController.button(11).whileTrue(
            commands2.cmd.runOnce(
                lambda: self.angulator.setAngle(self.limelight.speaker_angle())
            )
        )

        self.operatorController.button(11).whileFalse(
            commands2.cmd.runOnce(lambda: self.angulator.setAngle(0))
        )

        self.operatorController.button(5).whileTrue(
            commands2.cmd.runOnce(
                lambda: self.angulator.setAngle(angulatorConstants.angle_of_amp + 10)
            )
        )
        self.operatorController.button(5).whileFalse(
            commands2.cmd.runOnce(lambda: self.angulator.setAngle(0))
        )
        pass

    def get_autonomous_command(self) -> Command:
        bezierPoints = PathPlannerPath.bezierFromPoses(
            [
                Pose2d(0, 0, Rotation2d.fromDegrees(90)),
                Pose2d(0, 1, Rotation2d.fromDegrees(0)),
                Pose2d(1, 1, Rotation2d.fromDegrees(-90)),
                Pose2d(1, 0, Rotation2d.fromDegrees(180)),
                Pose2d(0, 0, Rotation2d.fromDegrees(90)),
            ]
        )

        # Create the path using the bezier points created above
        path = PathPlannerPath(
            bezierPoints,
            PathConstraints(
                3.0, 3.0, 2 * math.pi, 4 * math.pi
            ),  # The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            GoalEndState(
                0.0, Rotation2d.fromDegrees(0)
            ),  # Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        )
        path_follower_command: Command = AutoBuilder.followPath(path)
        command_group = SequentialCommandGroup()
        command_group.addCommands(
            commands2.InstantCommand(
                lambda: self.swerveSubsystem.resetOdometry(
                    Pose2d(0, 0, Rotation2d.fromDegrees(0))
                )
            )
        )
        command_group.addCommands(path_follower_command)
        command_group.addCommands(
            commands2.InstantCommand(lambda: self.swerveSubsystem.drive(0, 0, 0))
        )

        return command_group
