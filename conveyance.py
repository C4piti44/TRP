import commands2
from commands2 import Subsystem
from rev import CANSparkMax, CANSparkMaxLowLevel
from Constants import intakeConstants
from util.sparkmax_util import CANSparkMaxUtil, Usage

class intake(Subsystem):
    def __init__(self) -> None:
        # creating the motors
        self.right_motor: CANSparkMax = CANSparkMax(
            CANSparkMaxLowLevel.MotorType.kBrushless, intakeConstants.right_motor_id
        )
        self.left_motor: CANSparkMax = CANSparkMax(
            CANSparkMaxLowLevel.MotorType.kBrushless, intakeConstants.left_motor_id
        )
        self.config_motors() 

    def config_motors(self) -> None:
        self.right_motor.setIdleMode(
            CANSparkMax.IdleMode.kBrake
        )  # making it so that if the motor doesnt move he rejects all other outside movement
        self.left_motor.setIdleMode(
            CANSparkMax.IdleMode.kBrake
        )  # making it so that if the motor doesnt move he rejects all other outside movement
      
    def set_motors(self, power: float) -> None:
        if abs(power) > 1:  # saftey messure
            print("The Power Entered Into The Intake Motors Is Too High")
            return
        self.right_motor.set(-power)  # spinning the motors
        self.left_motor.set(power)
