from Constants import Constants
from Subsytem.SwerveSubsystem import SwerveSubsystem
import commands2
import commands2.cmd
import commands2.button
from Constants import Constants
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
from wpimath.geometry import Pose2d
from wpimath.controller import PIDController, ProfiledPIDControllerRadians
from commands2 import (
    SequentialCommandGroup,
    Swerve4ControllerCommand,
    Subsystem,
    Command,
)
import math


class RobotContainer:
    def __init__(self):
        #self.swerveSubsystem = SwerveSubsystem()

        self.driverController = commands2.button.CommandXboxController(
            Constants.OIConstants.kDriverControllerPort
        )
        #self.swerveSubsystem.setDefaultCommand(
        #    self.swerveSubsystem.run(
        #        lambda: SwerveSubsystem.drive(
        #            self.swerveSubsystem,
        #            self.driverController.getLeftX(),
        #            self.driverController.getLeftY(),
        #            self.driverController.getRightX(),
        #        )
        #    )
        #)
        #self.driverController.B().onTrue(
        #    commands2.cmd.runOnce(lambda: self.swerveSubsystem.zeroHeading())
        #)
        #self.configure_button_bindings()
        pass

    def configure_button_bindings(self):
        pass

    #def get_swerve(self) -> SwerveSubsystem:
    #    return self.swerveSubsystem

    #def print_joystick(self) -> None:
    #    print(f"Left Y: {str(self.driverController.getLeftY())}")
    #    print(f"Left X: {str(self.driverController.getLeftX())}")
    #    print(f"Right X: {str(self.driverController.getRightX())}")

    def get_autonomous_command(self) -> Command:
        config: TrajectoryConfig = TrajectoryConfig(
            Constants.Swerve.maxSpeed, Constants.Swerve.maxSpeed
        )
        config.setKinematics(Constants.Swerve.SwerveKinematics)

        pose2d_list: list[Pose2d] = list()
        pose2d_list.append(Pose2d(0, 0, 0))
        pose2d_list.append(Pose2d(-0.5, 0, 0))
        pose2d_list.append(Pose2d(0, -0.5, 0))

        trajectory = TrajectoryGenerator.generateTrajectory(
            pose2d_list,
            config,
        )

        theta_controller = ProfiledPIDControllerRadians(
            Constants.Swerve.thetaKP, 0, 0, Constants.Swerve.kThetaControllerConstraints
        )
        theta_controller.enableContinuousInput(-math.pi, math.pi)

        requirements: list[Subsystem] = list()
        requirements.append(self.swerveSubsystem)

        xController = PIDController(Constants.Swerve.kPXController, 0, 0)
        yController = PIDController(Constants.Swerve.kPYController, 0, 0)

        swerve_controller_command = Swerve4ControllerCommand(
            trajectory,
            self.swerveSubsystem.getPose,
            Constants.Swerve.SwerveKinematics,
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
