from rev import CANSparkMax
from wpimath.geometry import Translation2d
import math
import wpimath
import wpimath.kinematics

class Constants:
    class Swerve:
        stickDeadband = 0.1

        pigeonID = 42
        invertGyro = False  # Always ensure Gyro is CCW+ CW-

        # Drivetrain Constants #
        trackWidth = 0.59
        wheelBase = 0.59
        wheelDiameter = 4.0 * 0.0254
        wheelCircumference = wheelDiameter * math.pi

        openLoopRamp = 0.25
        closedLoopRamp = 0.0

        driveGearRatio = (6.75 / 1.0)
        angleGearRatio = ((150/7) / 1.0)

        mod0 = Translation2d(wheelBase / 2.0, trackWidth / 2.0)
        mod1 = Translation2d(wheelBase / 2.0, -trackWidth / 2.0)
        mod2 = Translation2d(-wheelBase / 2.0, trackWidth / 2.0)
        mod3 = Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)

        oldSwerveKinematics = wpimath.kinematics.SwerveDrive4Kinematics(
           mod0,
           mod1,
           mod2,
           mod3
        )


        # Swerve Voltage Compensation #
        voltageComp = 12.0

        # Swerve Current Limiting #
        angleContinuousCurrentLimit = 20
        driveContinuousCurrentLimit = 40

        # Angle Motor PID Values #
        angleKP = 0.1  # 0.12669
        angleKI = 0.0
        angleKD = 0.0
        angleKFF = 0.0

        # Drive Motor PID Values #
        driveKP = 3.3158E-05  # 1.6985E-18
        driveKI = 0.0
        driveKD = 0.0
        driveKFF = 0.0

        # Theta PID Values
        thetaKP = 0.01
        thetaKI = 0.0
        thetaKD = 0.008

        # Drive Motor Characterization Values #
        driveKS = 0.16548
        driveKV = 3.2091
        driveKA = 0.35702

        # Angle Motor Characterization Values #
        angleKS = 0.28394
        angleKV = 0.0044533
        angleKA = 0.00020852

        # Drive Motor Conversion Factors #
        driveConversionPositionFactor = (wheelDiameter * math.pi) / driveGearRatio
        driveConversionVelocityFactor = driveConversionPositionFactor / 60.0
        angleConversionFactor = 360.0 / angleGearRatio

        # Swerve Profiling Values #
        maxSpeed = 4.6
        maxAngularVelocity = 12

        maxAngularVelocityTele = 2 * math.pi
        angularVelocityScale = 3

        # Neutral Modes #
        angleNeutralMode = CANSparkMax.IdleMode.kBrake
        driveNeutralMode = CANSparkMax.IdleMode.kBrake

        # Motor Inverts ##
        driveInvert = False
        angleInvert = False

        # Angle Encoder Invert #
        canCoderInvert = False

    class DriveConstants:
        kPhysicalMaxSpeedMetersPerSecond = 4.6
        kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * math.pi

        kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4
        kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4
        kTeleDriveMaxAccelerationUnitsPerSecond = 3
        kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3

        #FrontLeft
        kFrontLeftDriveMotorPort = 2
        kFrontLeftTurningMotorPort = 1
        kFrontLeftTurningEncoderReversed = True
        kFrontLeftDriveEncoderReversed = False
        kFrontLeftDriveAbsoluteEncoderPort = 9
        kFrontLeftDriveAbsoluteEncoderReversed = False
        kFrontLeftDriveAbsoluteEncoderOffset = 35.244+180
    
        #FrontRight
        kFrontRightDriveMotorPort = 7
        kFrontRightTurningMotorPort = 8
        kFrontRightTurningEncoderReversed = True
        kFrontRightDriveEncoderReversed = True
        kFrontRightDriveAbsoluteEncoderPort = 12
        kFrontRightDriveAbsoluteEncoderReversed = False
        kFrontRightDriveAbsoluteEncoderOffset = 190.283

        #BackLeft
        kBackLeftDriveMotorPort = 3
        kBackLeftTurningMotorPort = 4
        kBackLeftTurningEncoderReversed = True
        kBackLeftDriveEncoderReversed = True
        kBackLeftDriveAbsoluteEncoderPort = 10
        kBackLeftDriveAbsoluteEncoderReversed = False
        kBackLeftDriveAbsoluteEncoderOffset = 149.50
    
        #BackRight
        kBackRightDriveMotorPort = 5
        kBackRightTurningMotorPort = 6
        kBackRightTurningEncoderReversed =  True
        kBackRightDriveEncoderReversed = True
        kBackRightDriveAbsoluteEncoderPort = 11
        kBackRightDriveAbsoluteEncoderReversed = False
        kBackRightDriveAbsoluteEncoderOffset = 129.463

    class OIConstants:
        kDriverControllerPort = 0

        #stick drift values
        kStickDriftLX = 0.1
        kStickDriftLY = 0.1
        kStickDriftRX = 0.1

