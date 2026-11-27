package first.subsystems.ElevatorSubsystem;

import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorConfig {
  public static final SparkMaxConfig ElevatorMotorConfig = new SparkMaxConfig();

  static {
    ElevatorMotorConfig.smartCurrentLimit(40).inverted(true);
  }
}
