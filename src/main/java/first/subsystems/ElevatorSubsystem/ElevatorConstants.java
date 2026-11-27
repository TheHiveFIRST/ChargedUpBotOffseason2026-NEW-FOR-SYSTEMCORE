package first.subsystems.ElevatorSubsystem;

import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;

public class ElevatorConstants {
  public static final int ELEVATOR_LEADER_ID = 9;
  public static final int ELEVATOR_FOLLOWER_ID = 10;

  public static final Constraints ELEVATOR_CONSTRAINTS = new Constraints(10, 5);

  public static final double ELEVATOR_P = 1.0;
  public static final double ELEVATOR_I = 0.0;
  public static final double ELEVATOR_D = 0.0;

  public static final double ELEVATOR_FEED_FORWARD =
      0.2; // Feedforward gain for the elevator to fight gravity

  public static final double ELEVATOR_MIN_POSITION = 0.0;
  public static final double ELEVATOR_LOW_SCORING_POSITION = 1.0;
  public static final double ELEVATOR_MID_SCORING_POSITION = 2.0;
  public static final double ELEVATOR_HIGH_SCORING_POSITION = 3.0;
  public static final double ELEVATOR_INTAKE_POSITION = 4.0; // Maximum position of the elevator
}
