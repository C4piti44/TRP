import wpilib
import math
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Odometry, SwerveDrive4Kinematics, ChassisSpeeds
from commands2 import Subsystem
import commands2
from Constants import (
    DriveConstants,
    OIConstants
    )
from Subsytem.SwerveModule import SwerveModule

class SwerveSubsystem(Subsystem):
    def __init__(self) -> None:
        commands2._impl.Subsystem.__init__(self)

        self.gyro = wpilib.ADXRS450_Gyro(wpilib.SPI.Port.kOnboardCS0)
        self.zeroHeading()
        
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
        self.odometer = SwerveDrive4Odometry(DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(self.gyro.getAngle()), [self.frontLeft.get_position(), self.frontRight.get_position(), self.backLeft.get_position(), self.backRight.get_position()])
        
        
    def zeroHeading(self) -> None:
        self.gyro.reset()

    def getHeading(self) -> float:
        return self.gyro.getAngle() % 360

    def getRotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.gyro.getAngle())

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
        xSpeed = xSpeed if abs(xSpeed) > OIConstants.kDeadband else 0.0
        ySpeed = ySpeed if abs(ySpeed) > OIConstants.kDeadband else 0.0
        tSpeed = tSpeed if abs(tSpeed) > OIConstants.kDeadband else 0.0

        translation: Translation2d = Translation2d(xSpeed, ySpeed)

        cSpeed: ChassisSpeeds
        if fieldOriented:
            cSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(translation.X(), translation.Y(), tSpeed, self.getRotation2d())
        else:
            cSpeed = ChassisSpeeds(xSpeed, ySpeed, tSpeed)

        moduleState = DriveConstants.kDriveKinematics.toSwerveModuleStates(cSpeed)
        self.setModuleStates(moduleState)

    def print_CANCoder_values(self):
        print(f"Encoder 0: {str(self.frontLeft.getAbsoluteEncoder())}")
        print(f"Encoder 1: {str(self.frontRight.getAbsoluteEncoder())}")
        print(f"Encoder 2: {str(self.backLeft.getAbsoluteEncoder())}")
        print(f"Encoder 3: {str(self.backRight.getAbsoluteEncoder())}")

    def print_NEO_encoder_values(self):
        print(f"Drive 0: {str(self.frontLeft.getDrivePosition())}")
        print(f"Turn 0: {str(self.frontLeft.getTurningPosition())}")
        print(f"Drive 1: {str(self.backLeft.getDrivePosition())}")
        print(f"Turn 1: {str(self.backLeft.getTurningPosition())}")
        print(f"Drive 2: {str(self.backRight.getDrivePosition())}")
        print(f"Turn 2: {str(self.backRight.getTurningPosition())}")
        print(f"Drive 3: {str(self.frontRight.getDrivePosition())}")
        print(f"Turn 3: {str(self.frontRight.getTurningPosition())}")
    
    def print_gyro_angle(self):
        print(f"Gyro Angle(Deg): {str(self.gyro.getAngle())}")