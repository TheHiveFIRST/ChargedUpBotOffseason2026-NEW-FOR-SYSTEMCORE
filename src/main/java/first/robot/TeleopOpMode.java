package first.robot;

import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

@Teleop
public class TeleopOpMode extends PeriodicOpMode {
  private final Robot robot;

  /** The Robot instance is passed into the opmode via the constructor. */
  public TeleopOpMode(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void disabledPeriodic() {
    /* Called on every robot period while the robot is disabled. */
  }

  @Override
  public void start() {
    /* Called once when the robot is enabled. */
  }

  @Override
  public void periodic() {
    robot.m_commandScheduler.run();
    /* Called on every robot period while the robot is enabled. */
  }

  @Override
  public void end() {
    /* Called when the robot is disabled (after previously being enabled). */
  }

  @Override
  public void close() {
    /* Called when the opmode is de-selected / no additional methods will be called. */
  }
}
