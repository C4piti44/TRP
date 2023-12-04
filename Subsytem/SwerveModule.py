from wpimath.controller import PIDController
from rev import CANSparkMax, CANSparkMaxLowLevel, SparkMaxPIDController
from ctre.sensors import CANCoder, CANCoderConfiguration, SensorInitializationStrategy, SensorTimeBase
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
import math
from Constants import (
    ModuleConstants,
    DriveConstants,
    )

class SwerveModule:
    def __init__(self, 
                driveMotorId:int, 
                turningMotorId:int,
                driveMotorReversed:bool,
                turningMotorReversed:bool,
                absoluteEncoderId:int,
                absoluteEncoderOffset:float,
                absoluteEncoderReversed:bool) -> None:
        
        self.absoluteEncoderOffset = absoluteEncoderOffset
        self.absoluteEncoderReversed = absoluteEncoderReversed
        self.absoluteEncoder = CANCoder(absoluteEncoderId)

        self.driveMotor = CANSparkMax(driveMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.turningMotor = CANSparkMax(turningMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)

        self.driveMotor.setInverted(driveMotorReversed)
        self.turningMotor.setInverted(turningMotorReversed)

        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = self.turningMotor.getEncoder()

        self.driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
        self.driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec)
        self.turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
        self.turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec)

        self.turningPidController = PIDController(ModuleConstants.kPTurning, 0, 0)
        self.turningPidController.enableContinuousInput(-math.pi, math.pi)

        self.config_absolute_encoder()
        self.resetEncoders()

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.getDrivePosition()*2*math.pi*ModuleConstants.kWheelDiameterMeters,
            Rotation2d(self.getAbsoluteEncoder())
        )

    def getDrivePosition(self) -> float:
        return self.driveEncoder.getPosition()

    def getTurningPosition(self) -> float:
        return self.turningEncoder.getPosition()

    def getDriveVelocity(self) -> float:
        return self.driveEncoder.getVelocity()

    def getTurningVelocity(self) -> float:
        return self.turningEncoder.getVelocity()

    def getAbsoluteEncoder(self) -> float:
        angle = self.absoluteEncoder.getAbsolutePosition()
        angle -= self.absoluteEncoderOffset
        return angle * (-1.0 if self.absoluteEncoderReversed else 1.0)

    def resetEncoders(self) -> None:
        self.driveEncoder.setPosition(0)
        self.turningEncoder.setPosition(self.getAbsoluteEncoder())

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.getDriveVelocity(), Rotation2d(self.getTurningPosition()))
    
    def config_absolute_encoder(self) -> None:
        self.absoluteEncoder.configFactoryDefault()
        swerve_can_coder_config = CANCoderConfiguration()
        swerve_can_coder_config.sensorDirection = False
        swerve_can_coder_config.initializationStrategy = (
            SensorInitializationStrategy.BootToAbsolutePosition
        )
        swerve_can_coder_config.sensorTimeBase = SensorTimeBase.PerSecond
        self.absoluteEncoder.configAllSettings(swerve_can_coder_config)

    def setDesiredState(self, state:SwerveModuleState) -> None:
        if abs(state.speed) < 0.01:
            self.stop()
            return
        state = SwerveModuleState.optimize(state, self.getState().angle)
        self.driveMotor.set(state.speed / DriveConstants.kPhysicalMaxSpeedMetersPerSecond)
        self.turningMotor.set(self.turningPidController.calculate(self.getTurningPosition()*(math.pi/180), state.angle.radians()))

    def stop(self) -> None:
        self.driveMotor.set(0)
        self.turningMotor.set(0)