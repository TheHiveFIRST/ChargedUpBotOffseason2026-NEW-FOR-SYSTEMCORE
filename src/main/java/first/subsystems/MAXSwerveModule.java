// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Configs;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.PIDController; // check this import if it doesn't resolve
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

public class MAXSwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  // Turning is closed-loop on the robot side now (wrap-around was removed from the SPARK).
  // Units: radians in, duty cycle out. Same gain as the old on-SPARK controller (P = 1).
  private final PIDController m_turningPID = new PIDController(0.4, 0, 0);

  private double m_chassisAngularOffset = 0;

  private SwerveModuleVelocity m_desiredState = new SwerveModuleVelocity(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motors, encoders, and PID
   * controllers.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkMax(CANPort.CAN_S0, drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(CANPort.CAN_S0, turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    m_drivingSpark.configure(
        Configs.MAXSwerveModule.drivingConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_turningSpark.configure(
        Configs.MAXSwerveModule.turningConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // The turning angle wraps every full rotation of the absolute encoder.
    m_turningPID.enableContinuousInput(0, 2 * Math.PI);

    m_chassisAngularOffset = chassisAngularOffset;

    m_desiredState =
        new SwerveModuleVelocity(0.0, new Rotation2d(getTurningRadians() - m_chassisAngularOffset));
    m_drivingEncoder.setPosition(0);
  }

  /** Absolute turning angle in radians, [0, 2*pi). */
  private double getTurningRadians() {
    return m_turningEncoder.getPosition().get() * Configs.MAXSwerveModule.kTurningPositionFactor;
  }

  /** Driving position in meters. */
  private double getDrivingMeters() {
    return m_drivingEncoder.getPosition().get() * Configs.MAXSwerveModule.kDrivingPositionFactor;
  }

  /** Driving velocity in meters per second. */
  private double getDrivingMetersPerSecond() {
    return m_drivingEncoder.getVelocity().get() * Configs.MAXSwerveModule.kDrivingVelocityFactor;
  }

  /** Returns the current velocity and angle of the module. */
  public SwerveModuleVelocity getState() {
    return new SwerveModuleVelocity(
        getDrivingMetersPerSecond(), new Rotation2d(getTurningRadians() - m_chassisAngularOffset));
  }

  /** Returns the current position of the module. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDrivingMeters(), new Rotation2d(getTurningRadians() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired velocity and angle for the module. Call this every loop, since the turning PID
   * runs here.
   */
  public void setDesiredState(SwerveModuleVelocity desiredState) {
    // Apply chassis angular offset.
    SwerveModuleVelocity correctedDesiredState =
        new SwerveModuleVelocity(
            desiredState.velocity,
            desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset)));

    // Optimize to minimize module rotation. The corrected state includes the offset,
    // so compare it against the raw encoder angle.
    correctedDesiredState = correctedDesiredState.optimize(new Rotation2d(getTurningRadians()));

    // Driving: convert m/s to the SPARK's native RPM.
    m_drivingClosedLoopController.setSetpoint(
        correctedDesiredState.velocity / Configs.MAXSwerveModule.kDrivingVelocityFactor,
        ControlType.kVelocity);

    // Turning: wrap-aware PID on the robot, output as duty cycle.
    double turnOutput =
        m_turningPID.calculate(getTurningRadians(), correctedDesiredState.angle.getRadians());
    m_turningClosedLoopController.setSetpoint(
        Math.max(-1.0, Math.min(1.0, turnOutput)), ControlType.kDutyCycle);

    m_desiredState = desiredState;
  }

  /** Zeroes the driving encoder. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
