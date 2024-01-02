import commands2
from commands2 import Subsystem
from rev import CANSparkMax, CANSparkMaxLowLevel
from Constants import elevatorConstants
from util.sparkmax_util import CANSparkMaxUtil, Usage
import math


class elevator(Subsystem):
    def __init__(self) -> None:
        commands2._impl.Subsystem.__init__(self)
        # creating the motor
        self.right_motor: CANSparkMax = CANSparkMax(
            CANSparkMaxLowLevel.MotorType.kBrushless, elevator.right_motor_id
        )
        self.left_motor: CANSparkMax = CANSparkMax(
            CANSparkMaxLowLevel.MotorType.kBrushless, elevator.left_motor_id
        )
        # getting the PID controllers from the motors(by doing that I can put the PID calculation on the SPARK instead of on the RoboRIO which helps with performance)
        self.right_controller = self.right_motor.getPIDController()
        self.left_controller = self.left_motor.getPIDController()
        self.encoder = (
            self.right_motor.getEncoder()
        )  # getting the encoder from the motor(doesnt matter which one)
        self.config()

    pass

    def config(self) -> None:
        # comments on uncommanted lines can be found in the intake.py file

        CANSparkMaxUtil.set_spark_max_bus_usage(self.right_motor, Usage.kMinimal)
        self.right_motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.right_motor.setInverted(True)
        self.right_controller.setP(
            elevatorConstants.kP
        )  # upadting the PID values of the SPARK
        self.right_controller.setI(elevatorConstants.kI)
        self.right_controller.setD(elevatorConstants.kD)

        CANSparkMaxUtil.set_spark_max_bus_usage(self.left_motor, Usage.kMinimal)
        self.left_motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.left_motor.setInverted(False)
        self.left_controller.setP(
            elevatorConstants.kP
        )  # updating the PID values of the SPARK
        self.left_controller.setI(elevatorConstants.kI)
        self.left_controller.setD(elevatorConstants.kD)

        # adding the gear ratio to the motor so he knows one motor spin doesnt mean one full spin for everything
        self.encoder.setPositionConversionFactor(elevatorConstants.conversion_factor)

    def move(self, position: float, override: bool = False) -> None:
        if override:
            if position >= 0.8:
                print(
                    "To Much Power Was Entered Into The Move Function In The elevator.pi File"
                )
                return
            self.right_motor.set(position)
            self.left_motor.set(position)
        else:
            angle = self.hight_to_angle(position)
            # spinning the motors until they reach the desired angle. position = angle
            self.right_controller.setReference(
                angle, CANSparkMaxLowLevel.ControlType.kPosition
            )
            self.left_controller.setReference(
                angle, CANSparkMaxLowLevel.ControlType.kPosition
            )

    def hight_to_angle(self, hight: float) -> float:
        angle = (hight * 360) / (2 * math.pi * elevatorConstants.radius_of_spool)
        return angle
