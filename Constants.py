from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d
import math
from wpimath.trajectory import TrapezoidProfileRadians


class IntakeConstants:
    motorID = 13
    intakePower = 0.5
    outtakePower = -0.5


class ConveyanceConstants:
    topMotorID = 11
    bottomMotorID = 12
    moveForwardPower = 0.75
    moveBackwardsPower = -0.75


class ShooterConstants:
    leftMotorID = 10
    rightMotorID = 9
    shootPower = 0.95


class ModuleConstants:
    driveKS = 0.16548
    driveKV = 3.2091
    driveKA = 0.35702
    # Angle Motor PID Values
    angle_kp = 0.01
    angle_ki = 0.0
    angle_kd = 0.0
    angle_kf = 0.0
    # Drive Motor PID Values
    drive_kp = 0.1
    drive_ki = 0.0
    drive_kd = 0.0
    drive_kf = 0.0

    kWheelDiameterMeters = 0.095
    kDriveMotorGearRatio = 6.75
    kTurningMotorGearRatio = 150 / 7
    kDriveEncoderRot2Meter = kDriveMotorGearRatio * math.pi * kWheelDiameterMeters
    kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * math.pi
    kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60
    kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60
    # Drive Motor Conversion Factors #
    driveConversionPositionFactor = (
        kWheelDiameterMeters * math.pi
    ) / kDriveMotorGearRatio
    driveConversionVelocityFactor = driveConversionPositionFactor / 60.0
    angleConversionFactor = 360.0 / kTurningMotorGearRatio
    kPTurning = 0.2


class DriveConstants:
    kTrackWidth = 0.75
    kWheelBase = 0.75
    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    )

    swerve_max_speed = 4

    kTeleDriveMaxAccelerationUnitsPerSecond = 3
    kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3
    # FrontLeft
    kFrontLeftDriveMotorPort = 2
    kFrontLeftTurningMotorPort = 1
    kFrontLeftTurningEncoderReversed = True
    kFrontLeftDriveEncoderReversed = False
    kFrontLeftDriveAbsoluteEncoderPort = 9
    kFrontLeftDriveAbsoluteEncoderReversed = False
    kFrontLeftDriveAbsoluteEncoderOffset = 208.125

    # FrontRight
    kFrontRightDriveMotorPort = 7
    kFrontRightTurningMotorPort = 8
    kFrontRightTurningEncoderReversed = True
    kFrontRightDriveEncoderReversed = False
    kFrontRightDriveAbsoluteEncoderPort = 12
    kFrontRightDriveAbsoluteEncoderReversed = False
    kFrontRightDriveAbsoluteEncoderOffset = 100.195

    # BackLeft
    kBackLeftDriveMotorPort = 3
    kBackLeftTurningMotorPort = 4
    kBackLeftTurningEncoderReversed = True
    kBackLeftDriveEncoderReversed = False
    kBackLeftDriveAbsoluteEncoderPort = 10
    kBackLeftDriveAbsoluteEncoderReversed = False
    kBackLeftDriveAbsoluteEncoderOffset = 330.820
    # BackRight
    kBackRightDriveMotorPort = 5
    kBackRightTurningMotorPort = 6
    kBackRightTurningEncoderReversed = True
    kBackRightDriveEncoderReversed = False
    kBackRightDriveAbsoluteEncoderPort = 11
    kBackRightDriveAbsoluteEncoderReversed = False
    kBackRightDriveAbsoluteEncoderOffset = 218.936
    kPhysicalMaxSpeedMetersPerSecond = 4.6
    kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * math.pi

    kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4
    kTeleDriveMaxAngularSpeedRadiansPerSecond = (
        kPhysicalMaxAngularSpeedRadiansPerSecond / 4
    )
    kTeleDriveMaxAccelerationUnitsPerSecond = 3
    kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3


class AutoConstants:
    kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4
    kMaxAngularSpeedRadiansPerSecond = (
        DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10
    )
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularAccelerationRadiansPerSecondSquared = math.pi / 4
    kPXController = 1.5
    kPYController = 1.5
    kPThetaController = 3
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared
    )


class OIConstants:
    kDriverControllerPort = 0

    # stick drift values
    kStickDriftLX = 0.1
    kStickDriftLY = 0.1
    kStickDriftRX = 0.1
