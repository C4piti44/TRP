
from wpimath import units
from wpimath.trajectory import TrapezoidProfile
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d

class ModuleConstants:
    kWheelDiameterMeters = units.inchesToMeters(4)
    kDriveMotorGearRatio = 1 / 5.8462
    kTurningMotorGearRatio = 1 / 18.0
    kDriveEncoderRot2Meter = kDriveMotorGearRatio * 3.141592653589793 * kWheelDiameterMeters
    kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * 3.141592653589793
    kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60
    kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60
    kPTurning = 0.5

class DriveConstants:
    kTrackWidth = units.inchesToMeters(21)
    kWheelBase = units.inchesToMeters(25.5)
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

    kFrontLeftTurningEncoderReversed = True
    kBackLeftTurningEncoderReversed = True
    kFrontRightTurningEncoderReversed = True
    kBackRightTurningEncoderReversed = True

    kFrontLeftDriveEncoderReversed = True
    kBackLeftDriveEncoderReversed = True
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

    kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254
    kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252
    kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816
    kBackRightDriveAbsoluteEncoderOffsetRad = -4.811

    kPhysicalMaxSpeedMetersPerSecond = 5
    kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * 3.141592653589793

    kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4
    kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4
    kTeleDriveMaxAccelerationUnitsPerSecond = 3
    kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3

class AutoConstants:
    kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4
    kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularAccelerationRadiansPerSecondSquared = 3.141592653589793 / 4
    kPXController = 1.5
    kPYController = 1.5
    kPThetaController = 3

    kThetaControllerConstraints = TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularAccelerationRadiansPerSecondSquared
    )

class OIConstants:
    kDriverControllerPort = 0

    kDriverYAxis = 1
    kDriverXAxis = 0
    kDriverRotAxis = 4
    kDriverFieldOrientedButtonIdx = 1

    kDeadband = 0.05
