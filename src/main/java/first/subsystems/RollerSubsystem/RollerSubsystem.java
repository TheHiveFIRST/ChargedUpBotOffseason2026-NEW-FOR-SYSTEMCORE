package first.subsystems.RollerSubsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;

public class RollerSubsystem extends SubsystemBase {
  // The SparkMax motor controller for the roller mechanism
  private final SparkMax m_rollerMotor;

  public RollerSubsystem() {
    // Initialize the SparkMax motor controller on CAN port 0 with a brushless motor type
    m_rollerMotor =
        new SparkMax(CANPort.CAN_S0, RollerConstants.kRollerMotorPort, MotorType.kBrushless);
    m_rollerMotor.configure(
        RollerConfig.RollerMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setRollerSpeed(double speed) {
    // Set the speed of the roller motor
    m_rollerMotor.setThrottle(speed);
  }

  // Stop the roller motor
  public void stopRoller() {
    // Stop the roller motor
    m_rollerMotor.setThrottle(0);
  }

  // run the rollers inward to intake the game piece
  public void Intake() {
    // Set the roller motor to intake speed
    m_rollerMotor.setThrottle(RollerConstants.kRollerIntakeSpeed); // Adjust the speed as necessary
  }

  // run the rollers outward to eject the game piece
  public void Outtake() {
    // Set the roller motor to outtake speed
    m_rollerMotor.setThrottle(RollerConstants.kRollerOuttakeSpeed); // Adjust the speed as necessary
  }

  // run the rollers inward at a slow speed to hold the game piece in place
  public void Hold() {
    // Set the roller motor to holding speed
    m_rollerMotor.setThrottle(RollerConstants.kRollerHoldSpeed); // Adjust the speed as necessary
  }

  public Command RollerIntakeCommand() {
    return run(
        () -> {
          Intake();
        });
  }

  public Command RollerOuttakeCommand() {
    return run(
        () -> {
          Outtake();
        });
  }

  public Command RollerStopCommand() {
    return run(
        () -> {
          stopRoller();
        });
  }

  public Command RollerHoldCommand() {
    return run(
        () -> {
          Hold();
        });
  }
}
