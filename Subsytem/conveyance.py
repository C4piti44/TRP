import commands2
import rev
from Constants import ConveyanceConstants


class Conveyance(commands2.Subsystem):
    def __init__(self) -> None:
        self.bottomMotor: rev.CANSparkMax = rev.CANSparkMax(
            ConveyanceConstants.bottomMotorID, rev.CANSparkMax.MotorType.kBrushless
        )
        self.topMotor: rev.CANSparkMax = rev.CANSparkMax(
            ConveyanceConstants.topMotorID, rev.CANSparkMax.MotorType.kBrushless
        )
        self.bottomMotor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        self.topMotor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)

        self.bottomMotor.setSmartCurrentLimit(20)
        self.topMotor.setSmartCurrentLimit(20)
        self.bottomMotor.enableVoltageCompensation(12)
        self.topMotor.enableVoltageCompensation(12)

    def move(self, power: float) -> None:
        if abs(power) > 1:
            print("Too Much Power Entered To The Conveyance Motors")
            return
        self.bottomMotor.set(power)
        self.topMotor.set(power)

    def amp(self, tPower: float, bPower: float) -> None:
        if abs(tPower) > 1 or abs(bPower) > 1:
            print("Too Much Power Entered To The Conveyance Motors")
            return
        self.bottomMotor.set(bPower)
        self.topMotor.set(tPower)
