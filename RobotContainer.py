from commands2 import Command, SequentialCommandGroup
from Constants import (
    OIConstants,
    DriveConstants,
    ModuleConstants,
    ConveyanceConstants,
    ShooterConstants,
    Intake,
)
from Subsytem.SwerveSubsystem import SwerveSubsystem
import commands2
import commands2.cmd
import commands2.button
import wpilib
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.config import (
    HolonomicPathFollowerConfig,
    PIDConstants,
    ReplanningConfig,
)
from pathplannerlib.auto import AutoBuilder
from Subsytem.Shooter import Shooter
from Subsytem.conveyance import Conveyance


class RobotContainer:
    def __init__(self):
        self.swerveSubsystem = SwerveSubsystem()
        self.shooter = Shooter()
        self.conveyance = Conveyance()

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
        # reset robots heading
        self.driverController.b().onTrue(
            commands2.cmd.runOnce(lambda: self.swerveSubsystem.zeroHeading())
        )

        # conveyance
        self.driverController.y().onTrue(
            commands2.cmd.runOnce(
                lambda: self.conveyance.move(ConveyanceConstants.moveForwardPower)
            )
        )
        self.driverController.y().onFalse(
            commands2.cmd.runOnce(lambda: self.conveyance.move(0))
        )

        self.driverController.a().onTrue(
            commands2.cmd.runOnce(
                lambda: self.conveyance.move(ConveyanceConstants.moveBackwardsPower)
            )
        )
        self.driverController.a().onFalse(
            commands2.cmd.runOnce(lambda: self.conveyance.move(0))
        )

        # shooter
        self.driverController.x().onTrue(
            commands2.cmd.runOnce(
                lambda: self.shooter.shoot(ShooterConstants.shootPower)
            )
        )
        self.driverController.x().onFalse(
            commands2.cmd.runOnce(lambda: self.shooter.shoot(0))
        )
        pass

    def get_autonomous_command(self) -> Command:
        config: HolonomicPathFollowerConfig = HolonomicPathFollowerConfig(
            PIDConstants(0.1, 0, 0),
            PIDConstants(0.1, 0, 0),
            DriveConstants.swerve_max_speed,
            ModuleConstants.kWheelDiameterMeters / 2,
            ReplanningConfig(),
        )

        AutoBuilder.configureHolonomic(
            self.swerveSubsystem.getPose,
            self.swerveSubsystem.resetOdometry,
            self.swerveSubsystem.getCSpeed,
            self.swerveSubsystem.updateAutoCSpeed,
            config,
            self.shouldFlipAutoPath,
            self.swerveSubsystem,
        )

        path: PathPlannerPath = PathPlannerPath.fromPathFile("Note1")
        path_follower_command: Command = AutoBuilder.followPath(path)
        command_group = SequentialCommandGroup()
        command_group.addCommands(path_follower_command)

        return command_group

    def getAlliance(self) -> wpilib.DriverStation.Alliance | None:
        return wpilib.DriverStation.getAlliance()

    def shouldFlipAutoPath(self) -> bool:
        color: wpilib.DriverStation.Alliance | None = self.getAlliance()
        return color != wpilib.DriverStation.Alliance.kBlue
