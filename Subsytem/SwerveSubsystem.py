import wpilib
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModulePosition, SwerveModuleState
from wpimath.estimator import SwerveDrive4PoseEstimator
from commands2 import Subsystem
import commands2
from wpimath import filter
from Constants import Constants
from Subsytem.SwerveModule import SwerveModule


class SwerveSubsystem(Subsystem):
    def __init__(self) -> None:
        commands2._impl.Subsystem.__init__(self)

        self.xLimiter = filter.SlewRateLimiter(
            Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond
        )
        self.yLimiter = filter.SlewRateLimiter(
            Constants.DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond
        )
        self.tLimiter = filter.SlewRateLimiter(
            Constants.DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond
            * 1.5
        )

        self.gyro = wpilib.ADXRS450_Gyro()
        self.zeroHeading()

        self.frontLeft: SwerveModule = SwerveModule(
            Constants.DriveConstants.kFrontLeftDriveMotorPort,
            Constants.DriveConstants.kFrontLeftTurningMotorPort,
            Constants.DriveConstants.kFrontLeftDriveEncoderReversed,
            Constants.DriveConstants.kFrontLeftTurningEncoderReversed,
            Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderOffset,
            Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
        )

        self.frontRight: SwerveModule = SwerveModule(
            Constants.DriveConstants.kFrontRightDriveMotorPort,
            Constants.DriveConstants.kFrontRightTurningMotorPort,
            Constants.DriveConstants.kFrontRightDriveEncoderReversed,
            Constants.DriveConstants.kFrontRightTurningEncoderReversed,
            Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderOffset,
            Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
        )

        self.backLeft: SwerveModule = SwerveModule(
            Constants.DriveConstants.kBackLeftDriveMotorPort,
            Constants.DriveConstants.kBackLeftTurningMotorPort,
            Constants.DriveConstants.kBackLeftDriveEncoderReversed,
            Constants.DriveConstants.kBackLeftTurningEncoderReversed,
            Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderOffset,
            Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
        )

        self.backRight: SwerveModule = SwerveModule(
            Constants.DriveConstants.kBackRightDriveMotorPort,
            Constants.DriveConstants.kBackRightTurningMotorPort,
            Constants.DriveConstants.kBackRightDriveEncoderReversed,
            Constants.DriveConstants.kBackRightTurningEncoderReversed,
            Constants.DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            Constants.DriveConstants.kBackRightDriveAbsoluteEncoderOffset,
            Constants.DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
        )
        self.odometer = SwerveDrive4PoseEstimator(
            Constants.Swerve.SwerveKinematics,
            Rotation2d.fromDegrees(self.getHeading()),
            (
                self.frontLeft.get_position(),
                self.frontRight.get_position(),
                self.backRight.get_position(),
                self.backLeft.get_position(),
            ),
            Pose2d(),
        )

    def get_modules_positions(self) -> tuple[SwerveModulePosition]:
        positions:tuple[SwerveModulePosition] = (
            self.frontLeft.get_position(),
            self.frontRight.get_position(),
            self.backLeft.get_position(),
            self.backRight.get_position()
        )
        return positions

    def zeroHeading(self) -> None:
        self.gyro.reset()

    def getHeading(self) -> float:
        angle: float = self.gyro.getAngle() % 360
        return 360 - angle

    def getRotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.getHeading())

    def getPose(self) -> Pose2d:
        return self.odometer.getEstimatedPosition()

    def update_odemeter(self) -> None:
        angle = self.getHeading()
        swerev_pose = (
            self.frontLeft.get_position(),
            self.frontRight.get_position(),
            self.backRight.get_position(),
            self.backLeft.get_position(),
        )

        self.odometer.update(angle, swerev_pose)

    def resetOdometry(self, pose: Pose2d) -> None:
        positions = self.get_modules_positions()
        self.odometer.resetPosition(self.getRotation2d(), positions, pose)

    def setModuleStates(self, desiredStates) -> None:
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, Constants.Swerve.maxSpeed
        )
        self.frontLeft.set_desired_state(desiredStates[0], True)
        self.frontRight.set_desired_state(desiredStates[1], True)
        self.backLeft.set_desired_state(desiredStates[2], True)
        self.backRight.set_desired_state(desiredStates[3], True)

    def drive(
        self, xSpeed: float, ySpeed: float, tSpeed: float, fieldOriented: bool = True
    ) -> None:
        xSpeed = xSpeed if abs(xSpeed) > Constants.OIConstants.kStickDriftLX else 0.0
        ySpeed = ySpeed if abs(ySpeed) > Constants.OIConstants.kStickDriftLY else 0.0
        tSpeed = tSpeed if abs(tSpeed) > Constants.OIConstants.kStickDriftRX else 0.0

        xSpeed = (
            self.xLimiter.calculate(xSpeed)
            * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
        )
        ySpeed = (
            self.yLimiter.calculate(ySpeed)
            * Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
        )
        tSpeed = (
            self.tLimiter.calculate(tSpeed)
            * Constants.DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond
            * 1.5
        )

        cSpeed: ChassisSpeeds
        if fieldOriented:
            cSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
                ySpeed, xSpeed, tSpeed, self.getRotation2d()
            )
        else:
            cSpeed = ChassisSpeeds(xSpeed, ySpeed, tSpeed)

        moduleState = Constants.Swerve.SwerveKinematics.toSwerveModuleStates(
            cSpeed, Translation2d()
        )
        self.setModuleStates(moduleState)
    
    def reset_modules(self):
        self.frontLeft.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(0)), True, False)
        self.frontRight.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(0)), True, False)
        self.backLeft.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(0)), True, False)
        self.backRight.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(0)), True, False)
