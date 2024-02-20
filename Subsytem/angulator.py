from commands2 import Subsystem
from rev import CANSparkMax
from Constants import angulatorConstants, LimeLightConstants
from util.sparkmax_util import CANSparkMaxUtil, Usage


class angulator(Subsystem):
    def __init__(self) -> None:
        # creating the motor
        self.right_motor: CANSparkMax = CANSparkMax(
            angulatorConstants.right_motor_id, CANSparkMax.MotorType.kBrushless
        )
        self.left_motor: CANSparkMax = CANSparkMax(
            angulatorConstants.left_motor_id, CANSparkMax.MotorType.kBrushless
        )

        self.right_controller = self.right_motor.getPIDController()
        self.left_controller = self.left_motor.getPIDController()
        self.encoder = self.right_motor.getEncoder()
        self.left_encoder = self.left_motor.getEncoder()
        self.config()
        self.angle = 0

    def config(self) -> None:
        self.right_motor.restoreFactoryDefaults()
        CANSparkMaxUtil.set_spark_max_bus_usage(self.right_motor, Usage.kAll)
        self.right_motor.setSmartCurrentLimit(40)
        self.right_motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.right_motor.setInverted(True)
        self.right_controller.setP(angulatorConstants.kP)
        self.right_controller.setI(angulatorConstants.kI)
        self.right_controller.setD(angulatorConstants.kD)
        self.right_controller.setFF(0)
        self.encoder.setPositionConversionFactor(angulatorConstants.conversion_factor)
        self.encoder.setPosition(0)
        self.right_motor.setOpenLoopRampRate(0.25)
        self.right_motor.enableVoltageCompensation(12)
        self.right_motor.burnFlash()

        self.left_motor.restoreFactoryDefaults()
        CANSparkMaxUtil.set_spark_max_bus_usage(self.left_motor, Usage.kAll)
        self.left_motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.left_motor.setInverted(False)
        self.left_controller.setP(angulatorConstants.kP)
        self.left_controller.setI(angulatorConstants.kI)
        self.left_controller.setD(angulatorConstants.kD)
        self.left_controller.setFF(0)
        self.left_encoder.setPositionConversionFactor(
            angulatorConstants.conversion_factor
        )
        self.left_encoder.setPosition(0)
        self.left_motor.setOpenLoopRampRate(0.25)
        self.left_motor.enableVoltageCompensation(12)
        self.left_motor.setSmartCurrentLimit(40)
        self.left_motor.burnFlash()

    def move(self, power: float) -> None:
        self.right_motor.set(power)
        self.left_motor.set(-power)

    def setAngle(self, angle: float = -0.1) -> None:
        if angle != -0.1:
            self.angle = angle
        self.set_angle(self.angle)

    def set_angle(self, angle: float) -> None:
        if abs(angle - self.encoder.getPosition()) > 1:
            self.right_controller.setReference(angle, CANSparkMax.ControlType.kPosition)
            self.left_controller.setReference(angle, CANSparkMax.ControlType.kPosition)
        else:
            self.right_motor.set(0)
            self.left_motor.set(0)
        LimeLightConstants.limelight_angle -= self.angle - self.encoder.getPosition()
