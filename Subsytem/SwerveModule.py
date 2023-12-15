from rev import CANSparkMax
from ctre.sensors import CANCoder
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from util.cancoder_util import CANCoderUtil, CCUsage
from util.sparkmax_util import CANSparkMaxUtil, Usage
from util.onboard_module_state import OnboardModuleState
from util.ctre_configs import CTREConfigs

from Constants import Constants


class SwerveModule:
    def __init__(
        self,
        drive_motor_id: int,
        turning_motor_id: int,
        drive_motor_reversed: bool,
        turning_motor_reversed: bool,
        absolute_encoder_id: int,
        absolute_encoder_offset: float,
        absolute_encoder_reversed: bool,
    ):
        self.absolute_encoder_reversed = absolute_encoder_reversed  # determining if the encoder is reversed or not
        self.feed_forward = SimpleMotorFeedforwardMeters(  #
            Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA
        )

        self.rotation_motor = CANSparkMax(turning_motor_id, CANSparkMax.MotorType.kBrushless)  # creating the rotation motor object
        self.drive_motor = CANSparkMax(drive_motor_id, CANSparkMax.MotorType.kBrushless)  # creating the driving motor object

        self.internal_rotation_encoder = self.rotation_motor.getEncoder()  # creating the encoder of the rotation motor
        self.angle_controller = self.rotation_motor.getPIDController()  # getting the PID object from the Spark

        self.drive_encoder = self.drive_motor.getEncoder()  # creating the encoder of the driving motor
        self.drive_controller = self.drive_motor.getPIDController()  # getting the PID object from the Spark

        self.angle_encoder = CANCoder(absolute_encoder_id)  # Creating the object of the CANCoder (the external encoder, which sits on the module).

        self.last_angle = self.internal_rotation_encoder.getPosition()  # getting the angle of the rotation motor

        self.desired_state = SwerveModuleState()  # setting the desired angle to the rotation motor

        self.angle_offset = absolute_encoder_offset  # declaring the offset of the module
        self.is_drive_motor_inverted = drive_motor_reversed  # determining if the driving motor is inverted
        self.is_turning_motor_inverted = turning_motor_reversed  # determining if the turning motor is inverted

        # confining the motors
        self.config_angle_encoder()
        self.config_angle_motor()
        self.config_drive_motor()

    def set_desired_state(self, desired_state: SwerveModuleState, is_open_loop: bool, override: bool = False,):
        desired_state = OnboardModuleState.optimize(desired_state, self.get_state().angle)
        self.set_angle(desired_state, override)
        self.set_speed(desired_state, is_open_loop)

    def reset_to_absolute(self):
        absolute_position = self.angle_encoder.getAbsolutePosition() - self.angle_offset
        desired_state = OnboardModuleState.optimize(
            SwerveModuleState(0, Rotation2d.fromDegrees(absolute_position)),
            self.get_state().angle,
            False,
        )

        self.internal_rotation_encoder.setPosition(desired_state.angle.degrees())

    def config_angle_encoder(self):
        self.angle_encoder.configFactoryDefault()
        CANCoderUtil.set_cancoder_bus_usage(self.angle_encoder, CCUsage.kMinimal)
        self.angle_encoder.configAllSettings(CTREConfigs.swerveCanCoderConfig)

    def config_angle_motor(self):
        # self.angle_motor.restoreFactoryDefaults()
        CANSparkMaxUtil.set_spark_max_bus_usage(self.rotation_motor, Usage.kPositionOnly)
        self.rotation_motor.setSmartCurrentLimit(
            Constants.Swerve.angleContinuousCurrentLimit
        )
        self.rotation_motor.setInverted(self.is_turning_motor_inverted)
        self.rotation_motor.setIdleMode(Constants.Swerve.angleNeutralMode)
        self.internal_rotation_encoder.setPositionConversionFactor(
            Constants.Swerve.angleConversionFactor
        )
        self.angle_controller.setP(Constants.Swerve.angleKP)
        self.angle_controller.setI(Constants.Swerve.angleKI)
        self.angle_controller.setD(Constants.Swerve.angleKD)
        self.angle_controller.setFF(Constants.Swerve.angleKFF)
        self.rotation_motor.setOpenLoopRampRate(0.0)
        self.rotation_motor.enableVoltageCompensation(Constants.Swerve.voltageComp)
        self.rotation_motor.burnFlash()
        self.reset_to_absolute()

    def config_drive_motor(self):
        # self.drive_motor.restoreFactoryDefaults()
        CANSparkMaxUtil.set_spark_max_bus_usage(self.drive_motor, Usage.kAll)
        self.drive_motor.setSmartCurrentLimit(
            Constants.Swerve.driveContinuousCurrentLimit
        )
        self.drive_motor.setInverted(self.is_drive_motor_inverted)
        self.drive_motor.setIdleMode(Constants.Swerve.driveNeutralMode)
        self.drive_encoder.setVelocityConversionFactor(
            Constants.Swerve.driveConversionVelocityFactor
        )
        self.drive_encoder.setPositionConversionFactor(
            Constants.Swerve.driveConversionPositionFactor
        )
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
                self.feed_forward.calculate(desired_state.speed),
            )

    def set_angle(self, desired_state: SwerveModuleState, override: bool = False):
        if (
            abs(desired_state.speed) <= (Constants.Swerve.maxSpeed * 0.01)
            and not override
        ):
            angle = self.last_angle
        else:
            angle = desired_state.angle.degrees()

        if abs(angle - self.internal_rotation_encoder.getPosition()) > 0.6:
            self.angle_controller.setReference(angle, CANSparkMax.ControlType.kPosition)
        else:
            self.rotation_motor.set(0)
        self.last_angle = angle

    def get_angle(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.internal_rotation_encoder.getPosition())

    def get_can_coder(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.angle_encoder.getAbsolutePosition())

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.drive_encoder.getPosition(),
            Rotation2d.fromDegrees(self.internal_rotation_encoder.getPosition()),
        )

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(self.drive_encoder.getVelocity(), self.get_angle())

    def stop_motors(self) -> None:
        self.rotation_motor.set(0)
        self.drive_motor.set(0)