
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.subsystems;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import org.wpilib.hardware.bus.CANPort;

import first.robot.Configs;

public class MAXSwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;

  private SwerveModuleVelocity m_desiredState =
      new SwerveModuleVelocity(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motors,
   * encoders, and PID controllers.
   */
  public MAXSwerveModule(
      int drivingCANId,
      int turningCANId,
      double chassisAngularOffset) {
      m_drivingSpark = new SparkMax(
          CANPort.CAN_S0,
          drivingCANId,
          MotorType.kBrushless
      );

      m_turningSpark = new SparkMax(
          CANPort.CAN_S0,
          turningCANId,
          MotorType.kBrushless
      );
    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController =
        m_drivingSpark.getClosedLoopController();

    m_turningClosedLoopController =
        m_turningSpark.getClosedLoopController();

    m_drivingSpark.configure(
        Configs.MAXSwerveModule.drivingConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_turningSpark.configure(
        Configs.MAXSwerveModule.turningConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;

   m_desiredState = new SwerveModuleVelocity(
        0.0,
        Rotation2d.fromRotations(
            m_turningEncoder.getPosition().get()
        ).minus(Rotation2d.fromRadians(m_chassisAngularOffset))
    ); 
  }
  /**
   * Returns the current velocity and angle of the module.
   */
 /**
 * Returns the current velocity and angle of the module.
 */
  public SwerveModuleVelocity getState() {

      return new SwerveModuleVelocity(
          m_drivingEncoder.getVelocity().get(),
          new Rotation2d(
              m_turningEncoder.getPosition().get() - m_chassisAngularOffset)
      );
  }

  /**
   * Returns the current position of the module.
   */
  public SwerveModulePosition getPosition() {

      return new SwerveModulePosition(
          m_drivingEncoder.getPosition().get(),
          new Rotation2d(
              m_turningEncoder.getPosition().get() - m_chassisAngularOffset)
      );
  }
  /**
   * Sets the desired velocity and angle for the module.
   */
  public void setDesiredState(SwerveModuleVelocity desiredState) {

    // Apply chassis angular offset.
    SwerveModuleVelocity correctedDesiredState =
        new SwerveModuleVelocity(
            desiredState.velocity,
            desiredState.angle.plus(
                Rotation2d.fromRadians(m_chassisAngularOffset)));

    // Optimize to minimize module rotation.
    correctedDesiredState =
        correctedDesiredState.optimize(
            new Rotation2d(m_turningEncoder.getPosition().get() - m_chassisAngularOffset));

    // Command drive and turning motors.
    m_drivingClosedLoopController.setSetpoint(
        correctedDesiredState.velocity,
        ControlType.kVelocity);

    m_turningClosedLoopController.setSetpoint(
        correctedDesiredState.angle.getRadians(),
        ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes the driving encoder. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
