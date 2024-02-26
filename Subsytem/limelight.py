from ntcore import NetworkTableInstance
from commands2 import Subsystem
from wpimath.controller import PIDController
from Constants import LimeLightConstants
import math


class limelight(Subsystem):
    def __init__(self) -> None:
        self.light = NetworkTableInstance.getDefault().getTable("limelight")
        self.controller = PIDController(0.02, 0, 0, 0)
        self.controller.enableContinuousInput(0, 360)

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
        return distance + 0.4

    def auto_align(self) -> float:
        if abs(self.getX()) < 1:
            return 0
        return self.controller.calculate(self.getX())
        

    def speaker_angle(self) -> float:
        d = self.distance()
        print(d)
        height = d/5
        temp = math.atan((1.3+height) / d) * (180 / math.pi)
        angle = (90-24)-temp
        print(angle)
        if angle > 60 or angle < 0:
            return 0 
        return angle
