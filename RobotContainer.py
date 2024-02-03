from commands2 import Command, SequentialCommandGroup
from Constants import OIConstants
from Subsytem.SwerveSubsystem import SwerveSubsystem
import commands2
import commands2.cmd
import commands2.button
import wpilib
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.commands import FollowPathCommand
from pathplannerlib.controller import PathFollowingController
from pathplannerlib.config import ReplanningConfig


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
        ppConfig: ReplanningConfig = ReplanningConfig(True, True, 0.2, 0.2)
        ppController: PathFollowingController = PathFollowingController()
        path: PathPlannerPath = PathPlannerPath.fromPathFile(
            "~/Documents/deploy/pathplanner/paths/Note 1.path"
        )
        path_follower_command: FollowPathCommand = FollowPathCommand(
            path,
            self.swerveSubsystem.getPose,
            self.swerveSubsystem.getCSpeed,
            self.swerveSubsystem.updateAutoCSpeed,
            ppController,
            ppConfig,
            self.shouldFlipAutoPath,
        )
        command_group = SequentialCommandGroup()
        command_group.addCommands(
            commands2.InstantCommand(
                lambda: self.swerveSubsystem.resetOdometry(
                    path.getStartingDifferentialPose()
                )
            )
        )
        command_group.addCommands(path_follower_command)

        return command_group

    def getAlliance(self) -> wpilib.DriverStation.Alliance | None:
        return wpilib.DriverStation.getAlliance()

    def shouldFlipAutoPath(self) -> bool:
        color: wpilib.DriverStation.Alliance | None = self.getAlliance()
        return color == wpilib.DriverStation.Alliance.kBlue
