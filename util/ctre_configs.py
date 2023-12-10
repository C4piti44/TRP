from ctre.sensors import CANCoderConfiguration, SensorTimeBase, AbsoluteSensorRange, SensorInitializationStrategy
from Constants import Constants


class CTREConfigs:
    swerveCanCoderConfig = CANCoderConfiguration()

    swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360
    swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert
    swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition
    swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond
