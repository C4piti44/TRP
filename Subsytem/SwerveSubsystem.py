from wpimath import filter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
from commands2 import Subsystem
from wpimath.estimator import SwerveDrive4PoseEstimator
from Constants import DriveConstants, OIConstants
from wpimath.geometry import Rotation2d
from Subsytem.SwerveModule import SwerveModule
from navx import AHRS
from pathplannerlib.config import (
    HolonomicPathFollowerConfig,
    PIDConstants,
    ReplanningConfig,
)
from pathplannerlib.auto import AutoBuilder
import wpilib
from Subsytem.limelight import limelight


class SwerveSubsystem(Subsystem):
    def __init__(self) -> None:
        self.autoCSpeed: ChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            0, 0, 0, Rotation2d.fromDegrees(0)
        )
        self.special_drive = False
        self.gyro = AHRS.create_i2c()
        self.zeroHeading()
        self.nt = limelight()

        self.xLimiter = filter.SlewRateLimiter(
            DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond
        )
        self.yLimiter = filter.SlewRateLimiter(
            DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond
        )
        self.tLimiter = filter.SlewRateLimiter(
            DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond
        )

        self.frontLeft: SwerveModule = SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffset,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
        )

        self.frontRight: SwerveModule = SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffset,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
        )

        self.backLeft: SwerveModule = SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffset,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
        )

        self.backRight: SwerveModule = SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffset,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
        )
        self.odometer = SwerveDrive4PoseEstimator(
            DriveConstants.kDriveKinematics,
            self.getRotation2d(),
            (
                self.frontLeft.get_position(),
                self.frontRight.get_position(),
                self.backLeft.get_position(),
                self.backRight.get_position(),
            ),
            Pose2d(),
        )

        AutoBuilder.configureHolonomic(
            self.getPose,  # Robot pose supplier
            self.resetOdometry,  # Method to reset odometry (will be called if your auto has a starting pose)
            self.getCSpeed,  # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.autoDrive,  # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig(  # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(5.0, 0.0, 0.0),  # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0),  # Rotation PID constants
                DriveConstants.swerve_max_speed,  # Max module speed, in m/s
                0.4,  # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig(),  # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath,  # Supplier to control path flipping based on alliance color
            self,  # Reference to this subsystem to set requirements
        )

    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

    def zeroHeading(self) -> None:
        self.gyro.reset()
        self.gyro.setAngleAdjustment(0)

    def autoHeading(self, angle: float) -> None:
        self.zeroHeading()
        self.gyro.setAngleAdjustment(angle)

    def getHeading(self) -> float:
        angle = self.gyro.getYaw() % 360
        return 360 - angle

    def getRotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.getHeading())

    def getPose(self) -> Pose2d:
        self.periodic()
        return self.odometer.getEstimatedPosition()

    def resetOdometry(self, pose: Pose2d) -> None:
        module_positions = (
            self.frontLeft.get_position(),
            self.frontRight.get_position(),
            self.backLeft.get_position(),
            self.backRight.get_position(),
        )
        self.odometer.resetPosition(self.getRotation2d(), module_positions, pose)

    def periodic(self) -> None:
        module_positions = (
            self.frontLeft.get_position(),
            self.frontRight.get_position(),
            self.backLeft.get_position(),
            self.backRight.get_position(),
        )
        self.odometer.update(self.getRotation2d(), module_positions)

    def setModuleStates(self, desiredStates) -> None:
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.swerve_max_speed
        )
        self.frontLeft.setDesiredState(desiredStates[3], True)
        self.frontRight.setDesiredState(desiredStates[0], True)
        self.backLeft.setDesiredState(desiredStates[1], True)
        self.backRight.setDesiredState(desiredStates[2], True)

    def drive(
        self, xSpeed: float, ySpeed: float, tSpeed: float, fieldOriented: bool = True
    ) -> None:
        xSpeed = xSpeed if abs(xSpeed) > OIConstants.kStickDriftLX else 0.0
        ySpeed = ySpeed if abs(ySpeed) > OIConstants.kStickDriftLY else 0.0
        tSpeed = tSpeed if abs(tSpeed) > OIConstants.kStickDriftRX else 0.0

        if self.special_drive:
            tSpeed = self.nt.auto_align()

        cSpeed: ChassisSpeeds
        if fieldOriented:
            cSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, tSpeed, self.getRotation2d()
            )
        else:
            cSpeed = ChassisSpeeds(xSpeed, ySpeed, tSpeed)

        moduleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            cSpeed, Translation2d()
        )
        self.setModuleStates(moduleState)

    def autoDrive(self, speed: ChassisSpeeds) -> None:
        temp = ChassisSpeeds.fromRobotRelativeSpeeds(speed, self.getRotation2d())
        moduleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            temp, Translation2d()
        )
        self.setModuleStates(moduleState)

    def getCSpeed(self) -> ChassisSpeeds:
        module_states = [
            self.frontLeft.getState(),
            self.frontRight.getState(),
            self.backRight.getState(),
            self.backLeft.getState(),
        ]
        return DriveConstants.kDriveKinematics.toChassisSpeeds(module_states)

    def change_drive(self, switch: bool):
        self.special_drive = switch

    def check_module_angle(self) -> None:
        self.frontLeft.reset_to_absolute()
        self.frontRight.reset_to_absolute()
        self.backLeft.reset_to_absolute()
        self.backRight.reset_to_absolute()
