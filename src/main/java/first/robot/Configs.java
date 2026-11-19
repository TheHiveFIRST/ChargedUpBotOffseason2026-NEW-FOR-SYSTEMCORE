package first.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import first.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        // REVLib alpha-7 removed conversion factors: the SPARK now reports native units
        // (rotations and RPM), so we convert in Java using these factors.

        /** Meters of wheel travel per driving motor rotation. */
        public static final double kDrivingPositionFactor =
                ModuleConstants.kWheelDiameterMeters * Math.PI
                        / ModuleConstants.kDrivingMotorReduction;
        /** Meters per second per driving motor RPM. */
        public static final double kDrivingVelocityFactor = kDrivingPositionFactor / 60.0;
        /** Radians per turning absolute-encoder rotation. */
        public static final double kTurningPositionFactor = 2 * Math.PI;

        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            double nominalVoltage = 12.0;
            // Volts per (m/s)
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Gains are now per RPM (native units). Old gains were per m/s, so scale them.
                    // These are example gains you may need to tune them for your own robot!
                    .pid(0.04 * kDrivingVelocityFactor, 0, 0)
                    .outputRange(-1, 1)
                    .feedForward.kV(drivingVelocityFeedForward * kDrivingVelocityFactor); // volts per RPM

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);

            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
                    .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

            // Turning position control (with wrap-around) is done in MAXSwerveModule with a
            // WPILib PIDController, because the SPARK's positionWrapping options were removed
            // in alpha-7. The SPARK just runs in duty-cycle mode for the turning motor.
        }
    }
}