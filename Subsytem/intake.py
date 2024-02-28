from commands2 import Subsystem
from rev import CANSparkMax
from Constants import IntakeConstants


class Intake(Subsystem):
    def __init__(self) -> None:
        self.motor: CANSparkMax = CANSparkMax(
            IntakeConstants.motorID, CANSparkMax.MotorType.kBrushless
        )
        self.motor.setSmartCurrentLimit(20)
        self.motor.enableVoltageCompensation(12)

    def move(self, power: float) -> None:
        if abs(power) > 1:
            print("To Much Power Entered To Intake Motor")
            return
        self.motor.set(power)
