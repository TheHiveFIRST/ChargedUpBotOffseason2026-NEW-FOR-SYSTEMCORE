package first.subsystems.ElevatorSubsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.subsystems.DriveSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ProfiledPIDController;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax m_elevatorMotorLeader;
  private final SparkMax m_elevatorMotorFollower;

  private final RelativeEncoder m_elevatorEncoder;

  private final ProfiledPIDController m_elevatorPIDController;

  private double m_targetPosition = 0.0; // Target position for the elevator

  private double m_currentPosition = 0.0; // Current position of the elevator

  public ElevatorSubsystem() {
    m_elevatorMotorLeader =
        new SparkMax(CANPort.CAN_S0, ElevatorConstants.ELEVATOR_LEADER_ID, MotorType.kBrushless);
    m_elevatorMotorFollower =
        new SparkMax(CANPort.CAN_S0, ElevatorConstants.ELEVATOR_FOLLOWER_ID, MotorType.kBrushless);

    m_elevatorPIDController =
        new ProfiledPIDController(
            ElevatorConstants.ELEVATOR_P,
            ElevatorConstants.ELEVATOR_I,
            ElevatorConstants.ELEVATOR_D,
            ElevatorConstants.ELEVATOR_CONSTRAINTS);

    m_elevatorPIDController.setTolerance(0.2, 0.2);

    m_elevatorMotorLeader.configure(
        ElevatorConfig.ElevatorMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_elevatorMotorFollower.configure(
        ElevatorConfig.ElevatorMotorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_elevatorEncoder = m_elevatorMotorLeader.getEncoder();
    m_elevatorEncoder.setPosition(m_currentPosition);
  }

  // Set the target position for the elevator
  public void setTargetPosition(double position) {
    m_targetPosition = position;
    if (m_targetPosition != ElevatorConstants.ELEVATOR_MIN_POSITION) {
      DriveSubsystem.isSlowMode =
          true; // Activate slow mode when the elevator is not at the minimum position
    }
  }

  //  Set the speed of the elevator motors
  public void setElevatorSpeed(double speed) {
    m_elevatorMotorLeader.setThrottle(speed);
    m_elevatorMotorFollower.setThrottle(-speed);
  }

  // Methods to retrieve information from the elevator subsystem
  // Get the current position of the elevator
  public double getCurrentPosition() {
    m_currentPosition = m_elevatorEncoder.getPosition().get();
    return m_currentPosition;
  }

  // Get the target position of the elevator
  public double getTargetPosition() {
    return m_targetPosition;
  }

  // Check if the elevator is at the target position
  public boolean isAtTargetPosition() {
    return m_elevatorPIDController.atGoal();
  }

  // Periodic method to update the elevator's position and control
  @Override
  public void periodic() {
    m_currentPosition = m_elevatorEncoder.getPosition().get();
    double output = m_elevatorPIDController.calculate(m_currentPosition, m_targetPosition);
    setElevatorSpeed(output + ElevatorConstants.ELEVATOR_FEED_FORWARD);
  }

  public Command MoveElevatorToPositionCommand(double position) {
    return run(
        () -> {
          setTargetPosition(position);
        });
  }
}
