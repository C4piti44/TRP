from commands2 import Command, Subsystem, Swerve4ControllerCommand, SequentialCommandGroup
from Constants import OIConstants
from Subsytem.SwerveSubsystem import SwerveSubsystem
import commands2
import commands2.cmd
import commands2.button
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.geometry import Pose2d
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from Constants import DriveConstants, ModuleConstants, AutoConstants
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
                    -self.driverController.getLeftY(),
                    -self.driverController.getLeftX(),
                    -self.driverController.getRightX(),
                )
            )
        )
        self.configure_button_bindings()

    def configure_button_bindings(self):
        self.driverController.B().onTrue(
            commands2.cmd.runOnce(
                lambda: self.swerveSubsystem.zeroHeading()
            )
        )

    def get_autonomous_command(self) -> Command:
        config: TrajectoryConfig = TrajectoryConfig(
            DriveConstants.swerve_max_speed, DriveConstants.swerve_max_speed
        )
        config.setKinematics(DriveConstants.kDriveKinematics)

        pose2d_list: list[Pose2d] = list()
        pose2d_list.append(Pose2d(0, 0, 0))
        pose2d_list.append(Pose2d(0.5, 0, 10))
        pose2d_list.append(Pose2d(0.5,0.5,20))

        trajectory = TrajectoryGenerator.generateTrajectory(
            pose2d_list,
            config,
        )

        theta_controller = ProfiledPIDControllerRadians(
            0.1, 0, 0, AutoConstants.kMaxAccelerationMetersPerSecondSquared
        )
        theta_controller.enableContinuousInput(-math.pi, math.pi)

        requirements: list[Subsystem] = list()
        requirements.append(self.swerveSubsystem)

        xController = PIDController(AutoConstants.kPXController, 0, 0)
        yController = PIDController(AutoConstants.kPYController, 0, 0)

        swerve_controller_command = Swerve4ControllerCommand(
            trajectory,
            self.swerveSubsystem.getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            theta_controller,
            self.swerveSubsystem.setModuleStates,
            requirements,
        )
        command_group = SequentialCommandGroup()
        command_group.addCommands(
            commands2.InstantCommand(
                lambda: self.swerveSubsystem.resetOdometry(trajectory.initialPose())
            )
        )
        command_group.addCommands(swerve_controller_command)

        return command_group

if __name__ == "__main__":
    # Instantiate RobotContainer and use its methods as needed
    robot_container = RobotContainer()
    auto_command = robot_container.get_autonomous_command()
    # ... Rest of your robot code ...
