
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

    kFrontLeftDriveMotorPort = 8
    kBackLeftDriveMotorPort = 2
    kFrontRightDriveMotorPort = 6
    kBackRightDriveMotorPort = 4

    kFrontLeftTurningMotorPort = 7
    kBackLeftTurningMotorPort = 1
    kFrontRightTurningMotorPort = 5
    kBackRightTurningMotorPort = 3

    kFrontLeftTurningEncoderReversed = False
    kBackLeftTurningEncoderReversed = False
    kFrontRightTurningEncoderReversed = False
    kBackRightTurningEncoderReversed =  False

    kFrontLeftDriveEncoderReversed = False
    kBackLeftDriveEncoderReversed = False
    kFrontRightDriveEncoderReversed = False
    kBackRightDriveEncoderReversed = False

    kFrontLeftDriveAbsoluteEncoderPort = 0
    kBackLeftDriveAbsoluteEncoderPort = 2
    kFrontRightDriveAbsoluteEncoderPort = 1
    kBackRightDriveAbsoluteEncoderPort = 3

    kFrontLeftDriveAbsoluteEncoderReversed = False
    kBackLeftDriveAbsoluteEncoderReversed = False
    kFrontRightDriveAbsoluteEncoderReversed = False
    kBackRightDriveAbsoluteEncoderReversed = False

    kFrontLeftDriveAbsoluteEncoderOffsetRad = 0
    kBackLeftDriveAbsoluteEncoderOffsetRad = 0
    kFrontRightDriveAbsoluteEncoderOffsetRad = 0
    kBackRightDriveAbsoluteEncoderOffsetRad = 0

    kPhysicalMaxSpeedMetersPerSecond = 5
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

    kDriverYAxis = 1
    kDriverXAxis = 0
    kDriverRotAxis = 4
    kDriverFieldOrientedButtonIdx = 1

    kDeadband = 0.05
