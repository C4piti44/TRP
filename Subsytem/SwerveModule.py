
from rev import CANSparkMax, CANSparkMaxLowLevel
from ctre.sensors import CANCoder
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
import math
from Constants import (
    ModuleConstants,
    DriveConstants,
    )

from util.cancoder_util import CANCoderUtil, CCUsage
from util.sparkmax_util import CANSparkMaxUtil, Usage
from util.onboard_module_state import OnboardModuleState
from util.ctre_configs import CTREConfigs

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

        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = self.turningMotor.getEncoder()

        self.driveMotorReversed = driveMotorReversed
        self.turningMotorReversed = turningMotorReversed

        self.turningPidController = self.turningMotor.getPIDController()

        self.config_rotation_encoder()
        self.config_driveMotor()
        self.config_turningMotor()

        self.last_angle = self.turningEncoder.getPosition()

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.driveEncoder.getPosition(),self.get_angle())

    # sync the internal motor encoder with the absolute encoder
    def reset_to_absolute(self):
        absolute_position = self.absoluteEncoder.getAbsolutePosition() - self.absoluteEncoderOffset
        desired_state = OnboardModuleState.optimize(
            SwerveModuleState(0, Rotation2d.fromDegrees(absolute_position)),
            self.getState().angle,
            False,
        )

        self.absoluteEncoder.setPosition(desired_state.angle.degrees())

    # setting up and configuring the rotation encoder
    def config_rotation_encoder(self):
        self.absoluteEncoder.configFactoryDefault()
        CANCoderUtil.set_cancoder_bus_usage(self.absoluteEncoder, CCUsage.kMinimal)
        self.absoluteEncoder.configAllSettings(CTREConfigs.swerveCanCoderConfig)

    # setting up the rotation motor and configuring it
    def config_turningMotor(self):
        self.turningMotor.restoreFactoryDefaults()
        CANSparkMaxUtil.set_spark_max_bus_usage(
            self.turningMotor, Usage.kPositionOnly
        )
        self.turningMotor.setSmartCurrentLimit(
            20
        )
        self.turningMotor.setInverted(self.turningMotorReversed)
        self.turningMotor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.turningEncoder.setPositionConversionFactor(
            ModuleConstants.angleConversionFactor
        )
        # configuring the PID
        self.turningPidController.setP(ModuleConstants.angle_kp)
        self.turningPidController.setI(ModuleConstants.angle_ki)
        self.turningPidController.setD(ModuleConstants.angle_kd)
        self.turningPidController.setFF(ModuleConstants.angle_kf)

        self.turningMotor.setOpenLoopRampRate(0.0)
        self.turningMotor.enableVoltageCompensation(12)
        self.turningMotor.burnFlash()
        self.reset_to_absolute()

    # setting up the driving motor and configuring it
    def config_driveMotor(self):
        self.driveMotor.restoreFactoryDefaults()
        CANSparkMaxUtil.set_spark_max_bus_usage(self.driveMotor, Usage.kAll)
        self.driveMotor.setSmartCurrentLimit(
            40
        )
        self.driveMotor.setInverted(self.driveMotorReversed)
        self.driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.driveEncoder.setVelocityConversionFactor(
            ModuleConstants.driveConversionVelocityFactor
        )
        self.driveEncoder.setPositionConversionFactor(
            ModuleConstants.driveConversionPositionFactor
        )
        self.driveEncoder.setPosition(0.0)
        self.driveMotor.setOpenLoopRampRate(0.25)

        self.driveMotor.burnFlash()
    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.driveEncoder.getVelocity(), self.get_angle())
    
    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.turningEncoder.getPosition())

    def setDesiredState(self, state:SwerveModuleState) -> None:
        if abs(state.speed) < 0.1:
            self.driveMotor.set(0)
        dState = OnboardModuleState.optimize(state, self.getState().angle)
        self.driveMotor.set(dState.speed / DriveConstants.kPhysicalMaxSpeedMetersPerSecond)
        if abs(self.turningEncoder.getPosition() - dState.angle.degrees()) > 1.5:
            self.turningPidController.setReference(dState.angle.degrees(), CANSparkMax.ControlType.kPosition)
        else:
            self.turningMotor.set(0)

    def stop(self) -> None:
        self.driveMotor.set(0)
        self.turningMotor.set(0)