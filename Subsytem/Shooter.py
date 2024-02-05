import commands2
import rev
from Constants import ShooterConstants


class Shooter(commands2.Subsystem):
    def __init__(self) -> None:
        self.leftMotor: rev.CANSparkMax = rev.CANSparkMax(
            ShooterConstants.leftMotorID, rev.CANSparkMax.MotorType.kBrushless
        )
        self.rightMotor: rev.CANSparkMax = rev.CANSparkMax(
            ShooterConstants.rightMotorID, rev.CANSparkMax.MotorType.kBrushless
        )

    def shoot(self, power: float):
        if abs(power) > 1:
            print("The Powered Entered Into The Shooting Motor Is Too High")
            return
        self.leftMotor.set(power)
        self.rightMotor.set(-power)
