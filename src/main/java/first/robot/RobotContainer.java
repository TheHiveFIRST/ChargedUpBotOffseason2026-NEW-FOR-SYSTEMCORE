// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import first.robot.Constants.OIConstants;
import first.subsystems.DriveSubsystem;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.RunCommand;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.button.JoystickButton;
import org.wpilib.driverstation.XboxController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // private final RollerSubsystem m_rollerSubsystem = new RollerSubsystem();
  // private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  // The driver's controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        -m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true,
                    true),
            m_robotDrive));
    // m_rollerSubsystem.setDefaultCommand(m_rollerSubsystem.RollerHoldCommand());
    // m_elevatorSubsystem.setDefaultCommand(
    //
    // m_elevatorSubsystem.MoveElevatorToPositionCommand(ElevatorConstants.ELEVATOR_MIN_POSITION));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link org.wpilib.driverstation.GenericHID} or one of its subclasses ({@link
   * org.wpilib.driverstation.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // drive subsystem button bindings
    m_driverController
        .y()
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    m_driverController.x().whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    m_driverController
        .dpadUp()
        .whileTrue(
            new RunCommand(
                () ->
                    m_robotDrive.driveHeadingLock(
                        -MathUtil.applyDeadband(
                            m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(
                            -m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                        true,
                        true,
                        Rotation2d.fromDegrees(0)),
                m_robotDrive));

    m_driverController
        .dpadDown()
        .whileTrue(
            new RunCommand(
                () -> m_robotDrive.driveToPos(0, 0, true, true, Rotation2d.fromDegrees(0)),
                m_robotDrive));

    /*
    // Roller subsystem button bindings
    m_driverController.b().whileTrue(m_roller
    
    Subsystem.RollerIntakeCommand());
    m_driverController.leftTrigger(0.3).whileTrue(m_rollerSubsystem.RollerIntakeCommand());

    // Elevator subsystem button bindings
    m_driverController
        .rightTrigger(0.3)
        .whileTrue(
            m_elevatorSubsystem.MoveElevatorToPositionCommand(
                ElevatorConstants.ELEVATOR_HIGH_SCORING_POSITION));
    m_driverController
        .rightBumper()
        .whileTrue(
            m_elevatorSubsystem.MoveElevatorToPositionCommand(
                ElevatorConstants.ELEVATOR_MID_SCORING_POSITION));
    m_driverController
        .leftBumper()
        .whileTrue(
            m_elevatorSubsystem.MoveElevatorToPositionCommand(
                ElevatorConstants.ELEVATOR_LOW_SCORING_POSITION));
    m_driverController
        .leftTrigger(0.3)
        .whileTrue(
            m_elevatorSubsystem.MoveElevatorToPositionCommand(
                ElevatorConstants.ELEVATOR_INTAKE_POSITION));
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
