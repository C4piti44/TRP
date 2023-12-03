
from wpimath import units
from wpimath.trajectory import TrapezoidProfile
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d
import math

class ModuleConstants:
    kWheelDiameterMeters = units.inchesToMeters(4)
    kDriveMotorGearRatio = 14/15
    kTurningMotorGearRatio = 14/45
    kDriveEncoderRot2Meter = kDriveMotorGearRatio * math.pi * kWheelDiameterMeters
    kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * math.pi
    kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60
    kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60
    kPTurning = 0.5

class DriveConstants:
    kTrackWidth = 0.59
    kWheelBase = 0.59
    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2)
    )

    swerve_max_speed = 1

    #FrontLeft
    kFrontLeftDriveMotorPort = 2
    kFrontLeftTurningMotorPort = 1
    kFrontLeftTurningEncoderReversed = False
    kFrontLeftDriveEncoderReversed = False
    kFrontLeftDriveAbsoluteEncoderPort = 9
    kFrontLeftDriveAbsoluteEncoderReversed = False
    kFrontLeftDriveAbsoluteEncoderOffset = 154.316
    
    #FrontRight
    kFrontRightDriveMotorPort = 7
    kFrontRightTurningMotorPort = 8
    kFrontRightTurningEncoderReversed = False
    kFrontRightDriveEncoderReversed = False
    kFrontRightDriveAbsoluteEncoderPort = 12
    kFrontRightDriveAbsoluteEncoderReversed = False
    kFrontRightDriveAbsoluteEncoderOffset = 5.361
    
    #BackLeft
    kBackLeftDriveMotorPort = 3
    kBackLeftTurningMotorPort = 4
    kBackLeftTurningEncoderReversed = False
    kBackLeftDriveEncoderReversed = False
    kBackLeftDriveAbsoluteEncoderPort = 10
    kBackLeftDriveAbsoluteEncoderReversed = False
    kBackLeftDriveAbsoluteEncoderOffset = 31.025
    
    #BackRight
    kBackRightDriveMotorPort = 5
    kBackRightTurningMotorPort = 6
    kBackRightTurningEncoderReversed =  False
    kBackRightDriveEncoderReversed = False
    kBackRightDriveAbsoluteEncoderPort = 11
    kBackRightDriveAbsoluteEncoderReversed = False
    kBackRightDriveAbsoluteEncoderOffset = 127.266

    kPhysicalMaxSpeedMetersPerSecond = 4.6
    kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * math.pi

    kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4
    kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4
    kTeleDriveMaxAccelerationUnitsPerSecond = 3
    kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3

class AutoConstants:
    kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4
    kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularAccelerationRadiansPerSecondSquared = math.pi / 4
#    kPXController = 1.5
#    kPYController = 1.5
#    kPThetaController = 3

#    kThetaControllerConstraints = TrapezoidProfile.Constraints(
#        kMaxAngularSpeedRadiansPerSecond,
#        kMaxAngularAccelerationRadiansPerSecondSquared
#    )

class OIConstants:
    kDriverControllerPort = 0

    #kDriverYAxis = 1
    #kDriverXAxis = 0
    #kDriverRotAxis = 4
    kDriverFieldOrientedButtonIdx = 1
    kDeadband = 0.05
