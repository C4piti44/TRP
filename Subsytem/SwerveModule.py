from wpimath.controller import PIDController
from rev import CANSparkMax, CANSparkMaxLowLevel
from wpilib import AnalogInput, PIDController
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState
import math
from Constants import (
    ModuleConstants,
    DriveConstants,
    )

class SwerveModule:
    def __init__(self, driveMotorId, turningMotorId, driveMotorReversed, turningMotorReversed,absoluteEncoderId, absoluteEncoderOffset, absoluteEncoderReversed) -> None:
        self.absoluteEncoderOffsetRad = absoluteEncoderOffset
        self.absoluteEncoderReversed = absoluteEncoderReversed
        self.absoluteEncoder = AnalogInput(absoluteEncoderId)

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
        self.turningPidController.enableContinuousInput(-math.pi, math.pis)

        self.resetEncoders()

    def getDrivePosition(self) -> float:
        return self.driveEncoder.getPosition()

    def getTurningPosition(self) -> float:
        return self.turningEncoder.getPosition()

    def getDriveVelocity(self) -> float:
        return self.driveEncoder.getVelocity()

    def getTurningVelocity(self) -> float:
        return self.turningEncoder.getVelocity()

    def getAbsoluteEncoderRad(self) -> float:
        angle = self.absoluteEncoder.getVoltage() / 5.0
        angle *= 2.0 * math.pi
        angle -= self.absoluteEncoderOffsetRad
        return angle * (-1.0 if self.absoluteEncoderReversed else 1.0)

    def resetEncoders(self) -> None:
        self.driveEncoder.setPosition(0)
        self.turningEncoder.setPosition(self.getAbsoluteEncoderRad())

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.getDriveVelocity(), Rotation2d(self.getTurningPosition()))
    
    def setDesiredState(self, state:SwerveModuleState) -> None:
        if abs(SwerveModuleState(state).speed) < 0.001:
            self.stop()
            return
        state = SwerveModuleState.optimize(state, self.getState().angle)
        self.driveMotor.set(state.speed / DriveConstants.kPhysicalMaxSpeedMetersPerSecond)
        self.turningMotor.set(self.turningPidController.calculate(self.getTurningPosition(), state.angle.getRadians()))

    def stop(self) -> None:
        self.driveMotor.set(0)
        self.turningMotor.set(0)