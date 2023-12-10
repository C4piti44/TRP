from rev import CANSparkMax
from ctre.sensors import CANCoder
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from util.cancoder_util import CANCoderUtil, CCUsage
from util.sparkmax_util import CANSparkMaxUtil, Usage
from util.swerve_module_consts import SwerveModuleConstants
from util.onboard_module_state import OnboardModuleState
from util.ctre_configs import CTREConfigs

from Constants import Constants


class SwerveModule:
    def __init__(self, 
                driveMotorId:int, 
                turningMotorId:int,
                driveMotorReversed:bool,
                turningMotorReversed:bool,
                absoluteEncoderId:int,
                absoluteEncoderOffset:float,
                absoluteEncoderReversed:bool):
        self.abosulteEncoderReversed = absoluteEncoderReversed
        self.feed_forward = SimpleMotorFeedforwardMeters(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA)

        self.angle_motor = CANSparkMax(turningMotorId,
                                       CANSparkMax.MotorType.kBrushless)
        self.drive_motor = CANSparkMax(driveMotorId,
                                       CANSparkMax.MotorType.kBrushless)

        self.integrated_angle_encoder = self.angle_motor.getEncoder()
        self.angle_controller = self.angle_motor.getPIDController()

        self.drive_encoder = self.drive_motor.getEncoder()
        self.drive_controller = self.drive_motor.getPIDController()

        self.angle_encoder = CANCoder(absoluteEncoderId)

        self.last_angle = self.integrated_angle_encoder.getPosition()

        self.desired_state = SwerveModuleState()

        self.angle_offset = absoluteEncoderOffset
        self.drive_invert = driveMotorReversed
        self.turn_invert = turningMotorReversed

        self.config_angle_encoder()
        self.config_angle_motor()
        self.config_drive_motor()

    def set_desired_state(self, desired_state: SwerveModuleState, is_open_loop: bool, override: bool = False):
        desiredState = OnboardModuleState.optimize(desired_state, self.get_state().angle)

        self.set_angle(desiredState, override)
        self.set_speed(desiredState, is_open_loop)

    def reset_to_absolute(self):
        absolute_position = self.angle_encoder.getAbsolutePosition() - self.angle_offset
        desired_state = OnboardModuleState.optimize(SwerveModuleState(0, Rotation2d.fromDegrees(absolute_position)), self.get_state().angle, False)

        self.integrated_angle_encoder.setPosition(desired_state.angle.degrees())

    def config_angle_encoder(self):
        self.angle_encoder.configFactoryDefault()
        CANCoderUtil.set_cancoder_bus_usage(self.angle_encoder, CCUsage.kMinimal)
        self.angle_encoder.configAllSettings(CTREConfigs.swerveCanCoderConfig)

    def config_angle_motor(self):
        # self.angle_motor.restoreFactoryDefaults()
        CANSparkMaxUtil.set_spark_max_bus_usage(self.angle_motor, Usage.kPositionOnly)
        self.angle_motor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit)
        self.angle_motor.setInverted(self.turn_invert)
        self.angle_motor.setIdleMode(Constants.Swerve.angleNeutralMode)
        self.integrated_angle_encoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor)
        self.angle_controller.setP(Constants.Swerve.angleKP)
        self.angle_controller.setI(Constants.Swerve.angleKI)
        self.angle_controller.setD(Constants.Swerve.angleKD)
        self.angle_controller.setFF(Constants.Swerve.angleKFF)
        self.angle_motor.setOpenLoopRampRate(0.0)
        self.angle_motor.enableVoltageCompensation(Constants.Swerve.voltageComp)
        self.angle_motor.burnFlash()
        self.reset_to_absolute()

    def config_drive_motor(self):
        # self.drive_motor.restoreFactoryDefaults()
        CANSparkMaxUtil.set_spark_max_bus_usage(self.drive_motor, Usage.kAll)
        self.drive_motor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit)
        self.drive_motor.setInverted(self.drive_invert)
        self.drive_motor.setIdleMode(Constants.Swerve.driveNeutralMode)
        self.drive_encoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor)
        self.drive_encoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor)
        self.drive_encoder.setPosition(0.0)
        self.drive_motor.setOpenLoopRampRate(Constants.Swerve.openLoopRamp)
        self.drive_controller.setP(Constants.Swerve.angleKP)
        self.drive_controller.setI(Constants.Swerve.angleKI)
        self.drive_controller.setD(Constants.Swerve.angleKD)
        self.drive_controller.setFF(Constants.Swerve.angleKFF)
        self.drive_motor.enableVoltageCompensation(Constants.Swerve.voltageComp)
        self.drive_motor.burnFlash()

    def set_speed(self, desired_state: SwerveModuleState, is_open_loop: bool):
        if is_open_loop:
          percent_output = desired_state.speed / Constants.Swerve.maxSpeed
          self.drive_motor.set(percent_output)
        else:
            self.drive_controller.setReference(
                desired_state.speed,
                CANSparkMax.ControlType.kVelocity,
                0,
                self.feed_forward.calculate(desired_state.speed)
            )

    def set_angle(self, desired_state: SwerveModuleState, override: bool = False):
        if abs(desired_state.speed) <= (Constants.Swerve.maxSpeed * 0.01) and not override:
            angle = self.last_angle
        else:
            angle = desired_state.angle.degrees()

        if abs(angle - self.integrated_angle_encoder.getPosition()) > 0.6:
            self.angle_controller.setReference(angle, CANSparkMax.ControlType.kPosition)
        else:
            self.angle_motor.set(0)
        self.last_angle = angle

    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.integrated_angle_encoder.getPosition())

    def get_can_coder(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.angle_encoder.getAbsolutePosition())

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.drive_encoder.getPosition(), Rotation2d.fromDegrees(self.integrated_angle_encoder.getPosition()))

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(self.drive_encoder.getVelocity(), self.get_angle())



