from commands2 import Command, SequentialCommandGroup, command
from Constants import (
    OIConstants,
    DriveConstants,
    ModuleConstants,
    ConveyanceConstants,
    ShooterConstants,
    IntakeConstants,
    angulatorConstants,
)
from Auto.wait import waitCommand
from Subsytem.SwerveSubsystem import SwerveSubsystem
import commands2
import commands2.cmd
import commands2.button
from pathplannerlib.path import (
    PathPlannerPath,
    PathConstraints,
    GoalEndState,
    RotationTarget,
)
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
        self.operatorController = commands2.button.CommandXboxController(
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

        self.driverController.x().onTrue(
            commands2.cmd.runOnce(lambda: self.swerveSubsystem.change_drive(True))
        )
        self.driverController.x().onFalse(
            commands2.cmd.runOnce(lambda: self.swerveSubsystem.change_drive(False))
        )
        self.driverController.y().onTrue(
            commands2.cmd.runOnce(lambda: self.swerveSubsystem.check_module_angle())
        )

        # conveyance
        self.operatorController.rightBumper().onTrue(
            commands2.cmd.runOnce(
                lambda: self.conveyance.move(ConveyanceConstants.moveForwardPower)
            )
        )
        self.operatorController.rightBumper().onFalse(
            commands2.cmd.runOnce(lambda: self.conveyance.move(0))
        )

        self.operatorController.leftBumper().onTrue(
            commands2.cmd.runOnce(
                lambda: self.conveyance.move(ConveyanceConstants.moveBackwardsPower)
            )
        )
        self.operatorController.leftBumper().onFalse(
            commands2.cmd.runOnce(lambda: self.conveyance.move(0))
        )

        self.operatorController.leftTrigger().onTrue(
            commands2.cmd.runOnce(
                lambda: self.conveyance.amp(
                    -ConveyanceConstants.moveForwardPower - 0.1,
                    ConveyanceConstants.moveBackwardsPower / 6,
                )
            )
        )
        self.operatorController.leftTrigger().onFalse(
            commands2.cmd.runOnce(lambda: self.conveyance.move(0))
        )

        # shooter
        self.operatorController.rightTrigger().onTrue(
            commands2.cmd.runOnce(
                lambda: self.shooter.shoot(-ShooterConstants.shootPower)
            )
        )
        self.operatorController.rightTrigger().onFalse(
            commands2.cmd.runOnce(lambda: self.shooter.shoot(0))
        )

        # intake
        self.operatorController.povUp().onTrue(
            commands2.cmd.runOnce(lambda: self.intake.move(IntakeConstants.intakePower))
        )
        self.operatorController.povUp().onFalse(
            commands2.cmd.runOnce(lambda: self.intake.move(0))
        )
        self.operatorController.povDown().onTrue(
            commands2.cmd.runOnce(
                lambda: self.intake.move(-IntakeConstants.intakePower)
            )
        )
        self.operatorController.povDown().onFalse(
            commands2.cmd.runOnce(lambda: self.intake.move(0))
        )

        # angulator
        self.operatorController.a().onTrue(
            commands2.cmd.runOnce(
                lambda: self.angulator.setAngle(angulatorConstants.angle_of_amp)
            )
        )
        self.operatorController.a().onFalse(
            commands2.cmd.runOnce(lambda: self.angulator.setAngle(0))
        )

        self.operatorController.y().onTrue(
            commands2.cmd.runOnce(
                lambda: self.angulator.setAngle(self.limelight.speaker_angle())
                # lambda: print(self.limelight.speaker_angle())
            )
        )
        self.operatorController.y().onFalse(
            commands2.cmd.runOnce(lambda: self.angulator.setAngle(0))
        )

        self.operatorController.x().onTrue(
            commands2.cmd.runOnce(lambda: self.angulator.setAngle(15))
        )
        self.operatorController.x().onFalse(
            commands2.cmd.runOnce(lambda: self.angulator.setAngle(0))
        )
        pass

    def get_autonomous_command(self) -> Command:
        bezierPoints = PathPlannerPath.bezierFromPoses(
            [
                Pose2d(16-0.94, 6.73, Rotation2d.fromDegrees(0)),
                Pose2d(16-2.88, 6.73, Rotation2d.fromDegrees(0)),
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
            # rotationTargets
        )
        path_follower_command: Command = AutoBuilder.followPath(path)
        command_group = SequentialCommandGroup()
        command_group.addCommands(
            commands2.cmd.runOnce(
                lambda: self.shooter.shoot(-ShooterConstants.shootPower)
            )
        )
        command_group.addCommands(
            commands2.cmd.runOnce(
                lambda: self.angulator.setAngle(15)
            )
        )
        command_group.addCommands(waitCommand(1))
        command_group.addCommands(
            commands2.cmd.runOnce(
                lambda: self.conveyance.move(ConveyanceConstants.moveForwardPower)
            )
        )
        command_group.addCommands(waitCommand(0.5))
        command_group.addCommands(commands2.cmd.runOnce(lambda: self.shooter.shoot(0)))
        command_group.addCommands(
            commands2.cmd.runOnce(lambda: self.intake.move(IntakeConstants.intakePower))
        )
        command_group.addCommands(
            commands2.InstantCommand(
                lambda: self.swerveSubsystem.resetOdometry(
                    Pose2d(16-0.94, 6.73, Rotation2d.fromDegrees(60))
                )
            )
        )
        #command_group.addCommands(path_follower_command)
        command_group.addCommands(commands2.cmd.runOnce(lambda: self.swerveSubsystem.autoHeading(60.0)))
        command_group.addCommands(commands2.cmd.runOnce(lambda: self.swerveSubsystem.drive(0.8, 0, 0)))
        command_group.addCommands(waitCommand(4))
        command_group.addCommands(
            commands2.InstantCommand(lambda: self.swerveSubsystem.drive(0, 0, 0))
        )
        command_group.addCommands(commands2.cmd.runOnce(lambda: self.intake.move(0)))
        command_group.addCommands(
            commands2.cmd.runOnce(lambda: self.conveyance.move(0))
        )
        command_group.addCommands(
            commands2.cmd.runOnce(lambda: self.angulator.setAngle(0))
        )
        return command_group
