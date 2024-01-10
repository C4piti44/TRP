from ctre.sensors import (
    CANCoderConfiguration,
    SensorTimeBase,
    AbsoluteSensorRange,
    SensorInitializationStrategy,
)

class CTREConfigs:
    swerveCanCoderConfig = CANCoderConfiguration()

    swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360
    swerveCanCoderConfig.sensorDirection = False
    swerveCanCoderConfig.initializationStrategy = (
        SensorInitializationStrategy.BootToAbsolutePosition
    )
    swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond
