from ntcore import NetworkTableInstance
from commands2 import Subsystem
from wpimath.controller import PIDController
from Constants import LimeLightConstants
import math


class limelight(Subsystem):
    def __init__(self) -> None:
        self.light = NetworkTableInstance.getDefault().getTable("limelight")
        self.controller = PIDController(0.1, 0, 0, 0)

    def getX(self) -> float:
        return self.light.getEntry("tx").getDouble(0.0)

    def getY(self) -> float:
        return self.light.getEntry("ty").getDouble(0.0)

    def distance(self) -> float:
        angle_to_goal_degrees = LimeLightConstants.limelight_angle + self.getY()
        angle_to_goal_radians = angle_to_goal_degrees * (3.14159 / 180)
        distance = (
            LimeLightConstants.target_height - LimeLightConstants.limelight_height
        ) / math.tan(angle_to_goal_radians)
        return distance

    def auto_align(self) -> float:
        return self.controller.calculate(self.getX())

    def speaker_angle(self) -> float:
        d = self.distance()
        angle = math.atan(2 / d) * (180 / math.pi)
        return angle
