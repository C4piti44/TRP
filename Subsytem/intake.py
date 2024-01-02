import commands2
from commands2 import Subsystem
from rev import CANSparkMax, CANSparkMaxLowLevel
from Constants import intakeConstants
from util.sparkmax_util import CANSparkMaxUtil, Usage


class intake(Subsystem):
    def __init__(self) -> None:
        commands2._impl.Subsystem.__init__(self)
        # creating the motors
        self.right_motor: CANSparkMax = CANSparkMax(
            CANSparkMaxLowLevel.MotorType.kBrushless, intakeConstants.right_motor_id
        )
        self.left_motor: CANSparkMax = CANSparkMax(
            CANSparkMaxLowLevel.MotorType.kBrushless, intakeConstants.left_motor_id
        )
        self.config_motors()

    pass

    def config_motors(self) -> None:
        CANSparkMaxUtil.set_spark_max_bus_usage(
            self.right_motor, Usage.kMinimal
        )  # setting the CANBUS to a minimal input to the SPARK
        self.right_motor.setIdleMode(
            CANSparkMax.IdleMode.kBrake
        )  # making it so that if the motor doesnt move he rejects all other outside movement
        self.right_motor.setInverted(True)  # changing the direction of the motor

        CANSparkMaxUtil.set_spark_max_bus_usage(
            self.left_motor, Usage.kMinimal
        )  # setting the CANBUS to a minimal input to the SPARK
        self.left_motor.setIdleMode(
            CANSparkMax.IdleMode.kBrake
        )  # making it so that if the motor doesnt move he rejects all other outside movement
        self.left_motor.setInverted(False)  # NOT changing the direction of the motor

    def set_motors(self, power: float) -> None:
        if abs(power) > intakeConstants.speed_limit:  # saftey messure
            print("The Power Entered Into The Intake Motors Is Too High")
            return
        self.right_motor.set(power)  # spinning the motors
        self.left_motor.set(power)
