import wpilib
from wpimath import filter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Odometry, SwerveDrive4Kinematics, ChassisSpeeds
from commands2 import Subsystem
import commands2
import math
from Constants import (
    DriveConstants,
    OIConstants
    )
from Subsytem.SwerveModule import SwerveModule

class SwerveSubsystem(Subsystem):
    def __init__(self) -> None:
        commands2._impl.Subsystem.__init__(self)

        self.gyro = wpilib.ADXRS450_Gyro()
        self.zeroHeading()

        self.xLimiter = filter.SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond)
        self.yLimiter = filter.SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond)
        self.tLimiter = filter.SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond)

        self.frontLeft:SwerveModule = SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffset,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
        )

        self.frontRight:SwerveModule = SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffset,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
        )

        self.backLeft:SwerveModule = SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffset,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
        )

        self.backRight:SwerveModule = SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffset,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed
        )
        self.odometer = SwerveDrive4Odometry(DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(self.gyro.getAngle()),[self.frontLeft.get_position(), self.frontRight.get_position(), self.backLeft.get_position(), self.backRight.get_position()])
        
        
    def zeroHeading(self) -> None:
        self.gyro.reset()

    def getHeading(self) -> float:
        angle = self.gyro.getAngle()%360
        return 360 - angle

    def getRotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.getHeading())

    def getPose(self) -> Pose2d:
        return self.odometer.getPose()

    def resetOdometry(self, pose:Pose2d) -> None:
        self.odometer.resetPosition(pose, self.getRotation2d())

    def periodic(self) -> None:
        self.odometer.update(
            self.getRotation2d(),
            self.frontLeft.get_position(),
            self.frontRight.get_position(),
            self.backLeft.get_position(),
            self.backRight.get_position()
        )

    def stopModules(self) -> None:
        self.frontLeft.stop()
        self.frontRight.stop()
        self.backLeft.stop()
        self.backRight.stop()

    def setModuleStates(self, desiredStates) -> None:
        SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond)
        self.frontLeft.setDesiredState(desiredStates[0])
        self.frontRight.setDesiredState(desiredStates[1])
        self.backLeft.setDesiredState(desiredStates[2])
        self.backRight.setDesiredState(desiredStates[3])

    def drive(self, xSpeed: float, ySpeed: float, tSpeed: float, fieldOriented: bool = True) -> None:
        xSpeed = xSpeed if abs(xSpeed) > OIConstants.kStickDriftLX else 0.0
        ySpeed = ySpeed if abs(ySpeed) > OIConstants.kStickDriftLY else 0.0
        tSpeed = tSpeed if abs(tSpeed) > OIConstants.kStickDriftRX else 0.0

        xSpeed = self.xLimiter.calculate(xSpeed)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
        ySpeed = self.yLimiter.calculate(ySpeed)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
        tSpeed = self.tLimiter.calculate(tSpeed)*DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond

        cSpeed: ChassisSpeeds
        if fieldOriented:
            cSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, tSpeed, self.getRotation2d())
        else:
            cSpeed = ChassisSpeeds(xSpeed, ySpeed, tSpeed)

        moduleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(cSpeed, Translation2d())
        self.setModuleStates(moduleState)