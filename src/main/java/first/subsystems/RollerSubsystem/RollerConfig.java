package first.subsystems.RollerSubsystem;

import com.revrobotics.spark.config.SparkMaxConfig;

public class RollerConfig {
  public static final SparkMaxConfig RollerMotorConfig = new SparkMaxConfig();

  static {
    RollerMotorConfig.smartCurrentLimit(40).inverted(true);
  }
}
