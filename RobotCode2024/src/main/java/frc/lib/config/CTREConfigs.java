package frc.lib.config;


import com.ctre.phoenix6.configs.CANcoderConfiguration;

public final class CTREConfigs {
  public CANcoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveCanCoderConfig = new CANcoderConfiguration();

    // /* Swerve CANCoder Configuration */
    // swerveCanCoderConfig.MagnetSensor = MagnetSensorConfigs;
    // swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
    // swerveCanCoderConfig.initializationStrategy =
    //     SensorInitializationStrategy.BootToAbsolutePosition;
    // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  }
}
