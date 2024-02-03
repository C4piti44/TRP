import wpilib
from wpimath import filter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
from commands2 import Subsystem
from wpimath.estimator import SwerveDrive4PoseEstimator
from Constants import DriveConstants, OIConstants
from wpimath.geometry import Rotation2d
from Subsytem.SwerveModule import SwerveModule


class SwerveSubsystem(Subsystem):
    def __init__(self) -> None:
        self.autoCSpeed: ChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            0, 0, 0, Rotation2d.fromDegrees(0)
        )
        self.gyro = wpilib.ADXRS450_Gyro()
        self.zeroHeading()

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

    def zeroHeading(self) -> None:
        self.gyro.reset()

    def getHeading(self) -> float:
        angle = self.gyro.getAngle() % 360
        return 360 - angle

    def getRotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.getHeading())

    def getPose(self) -> Pose2d:
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
            desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond
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

        cSpeed: ChassisSpeeds
        if fieldOriented:
            cSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, tSpeed, self.getRotation2d()
            )
        else:
            cSpeed = ChassisSpeeds(xSpeed, ySpeed, tSpeed)

        self.autoCSpeed = ChassisSpeeds(xSpeed, ySpeed, tSpeed)

        moduleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            cSpeed, Translation2d()
        )
        self.setModuleStates(moduleState)

    def autoDrive(self, speed: ChassisSpeeds):
        moduleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            speed, Translation2d()
        )
        self.setModuleStates(moduleState)

    def getCSpeed(self) -> ChassisSpeeds:
        return self.autoCSpeed

    def updateAutoCSpeed(self, speed: ChassisSpeeds) -> None:
        self.autoCSpeed = speed
        self.autoDrive(speed)
