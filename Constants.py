
from wpimath.trajectory import TrapezoidProfile
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Translation2d
import math

class ModuleConstants:
    
    # Angle Motor PID Values
    angle_kp = 1
    angle_ki = 0.0
    angle_kd = 0
    angle_kf = 0.0
    # Drive Motor PID Values
    drive_kp = 0.1
    drive_ki = 0.0
    drive_kd = 0.0
    drive_kf = 0.0

    kWheelDiameterMeters = 0.095
    kDriveMotorGearRatio = 6.75
    kTurningMotorGearRatio = 150/7
    kDriveEncoderRot2Meter = kDriveMotorGearRatio * math.pi * kWheelDiameterMeters
    kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * math.pi
    kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60
    kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60
    # Drive Motor Conversion Factors #
    driveConversionPositionFactor = (kWheelDiameterMeters * math.pi) / kDriveMotorGearRatio
    driveConversionVelocityFactor = driveConversionPositionFactor / 60.0
    angleConversionFactor = 360.0 / kTurningMotorGearRatio
    kPTurning = 0.2

class DriveConstants:
    kTrackWidth = 0.725
    kWheelBase = 0.725
    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    )

    swerve_max_speed = 4.6

    #FrontLeft
    kFrontLeftDriveMotorPort = 2
    kFrontLeftTurningMotorPort = 1
    kFrontLeftTurningEncoderReversed = True
    kFrontLeftDriveEncoderReversed = True
    kFrontLeftDriveAbsoluteEncoderPort = 9
    kFrontLeftDriveAbsoluteEncoderReversed = False
    kFrontLeftDriveAbsoluteEncoderOffset = 38.453
    
    #FrontRight
    kFrontRightDriveMotorPort = 7
    kFrontRightTurningMotorPort = 8
    kFrontRightTurningEncoderReversed = True
    kFrontRightDriveEncoderReversed = True
    kFrontRightDriveAbsoluteEncoderPort = 12
    kFrontRightDriveAbsoluteEncoderReversed = False
    kFrontRightDriveAbsoluteEncoderOffset = 185.449

    #BackLeft
    kBackLeftDriveMotorPort = 3
    kBackLeftTurningMotorPort = 4
    kBackLeftTurningEncoderReversed = True
    kBackLeftDriveEncoderReversed = True
    kBackLeftDriveAbsoluteEncoderPort = 10
    kBackLeftDriveAbsoluteEncoderReversed = False
    kBackLeftDriveAbsoluteEncoderOffset = 152.623
    
    #BackRight
    kBackRightDriveMotorPort = 5
    kBackRightTurningMotorPort = 6
    kBackRightTurningEncoderReversed =  True
    kBackRightDriveEncoderReversed = True
    kBackRightDriveAbsoluteEncoderPort = 11
    kBackRightDriveAbsoluteEncoderReversed = False
    kBackRightDriveAbsoluteEncoderOffset = 129.11

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
    kPXController = 1.5
    kPYController = 1.5
    kPThetaController = 3

    kThetaControllerConstraints = TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularAccelerationRadiansPerSecondSquared
    )

class OIConstants:
    kDriverControllerPort = 0

    #stick drift values
    kStickDriftLX = 0.15
    kStickDriftLY = 0.15
    kStickDriftRX = 0.15
