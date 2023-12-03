from wpilib import SPI
from wpimath import filter
from navx import AHRS
import wpilib

from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Odometry, SwerveDrive4Kinematics, ChassisSpeeds
from commands2 import SubsystemBase
import commands2
from Constants import (
    DriveConstants,
    OIConstants
    )
from Subsytem.SwerveModule import SwerveModule

class SwerveSubsystem(SubsystemBase):
    def __init__(self) -> None:

        commands2._impl.SubsystemBase.__init__(self)
        self.xLimiter = filter.SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond)
        self.yLimiter = filter.SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond)
        self.tLimiter = filter.SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond)
        
        self.frontLeft = SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
        )

        self.frontRight = SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
        )

        self.backLeft = SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
        )

        self.backRight = SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed
        )
        #self.gyro = AHRS(SPI.Port.kMXP)
        #self.gyro = wpilib.ADXRS450_Gyro
        self.gyro = wpilib.ADXRS450_Gyro(wpilib.SPI.Port.kOnboardCS0)
        self.odometer = SwerveDrive4Odometry(DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(self.gyro.getAngle()), [self.frontLeft.get_position(), self.frontRight.get_position(), self.backLeft.get_position(), self.backRight.get_position()])
        
    def zeroHeading(self) -> None:
        self.gyro.reset()

    def get_angle(self) -> float:
        return self.gyro.getAngle()

    def getHeading(self) -> float:
        return self.gyro.getAngle() % 360

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

    def drive(self,xSpeed:float, ySpeed:float, tSpeed:float, fieldOriented:bool) -> None:
        xSpeed = xSpeed if abs(xSpeed) > OIConstants.kDeadband else 0.0
        ySpeed = ySpeed if abs(ySpeed) > OIConstants.kDeadband else 0.0
        tSpeed = tSpeed if abs(tSpeed) > OIConstants.kDeadband else 0.0

        xSpeed = self.xLimiter.calculate(xSpeed)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
        ySpeed = self.yLimiter.calculate(ySpeed)*DriveConstants.kTeleDriveMaxSpeedMetersPerSecond
        tSpeed = self.tLimiter.calculate(tSpeed)*DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond

        cSpeed:ChassisSpeeds
        if fieldOriented:
            cSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,tSpeed,self.getRotation2d())
        else:
            cSpeed = ChassisSpeeds(xSpeed,ySpeed,tSpeed)

        moduleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(cSpeed)
        self.setModuleStates(moduleState)

    def print_CANCoder_values(self):
        print(f"Encoder 0: {str(self.frontLeft.getAbsoluteEncoderRad())}")
        print(f"Encoder 1: {str(self.frontRight.getAbsoluteEncoderRad())}")
        print(f"Encoder 2: {str(self.backLeft.getAbsoluteEncoderRad())}")
        print(f"Encoder 3: {str(self.backRight.getAbsoluteEncoderRad())}")

    def print_NEO_encoder_values(self):
        print(f"Drive 0: {str(self.frontLeft.getDrivePosition())}")
        print(f"Turn 0: {str(self.frontRight.getTurningPosition())}")
        print(f"Drive 1: {str(self.backLeft.getDrivePosition())}")
        print(f"Turn 1: {str(self.backRight.getTurningPosition())}")
        print(f"Drive 2: {str(self.frontLeft.getDrivePosition())}")
        print(f"Turn 2: {str(self.frontRight.getTurningPosition())}")
        print(f"Drive 3: {str(self.backLeft.getDrivePosition())}")
        print(f"Turn 3: {str(self.backRight.getTurningPosition())}")
        