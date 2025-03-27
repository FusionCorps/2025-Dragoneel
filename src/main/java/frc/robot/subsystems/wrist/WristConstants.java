package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;

public class WristConstants {
  public static final int WRIST_MOTOR_ID = 15;

  public static final AbsoluteEncoderConfig WRIST_ABSOLUTE_ENCODER_CONFIG =
      new AbsoluteEncoderConfig().zeroCentered(true);

  public static final ClosedLoopConfig WRIST_CLOSED_LOOP_CONFIG =
      new ClosedLoopConfig()
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .positionWrappingInputRange(-0.5, 0.5)
          .positionWrappingEnabled(true)
          .pid(6.0, 0, 0)
          .apply(
              new MAXMotionConfig()
                  .allowedClosedLoopError(0.005)
                  .maxVelocity(100.0)
                  .maxAcceleration(100.0));

  public static final double WRIST_MAX_MOTION_MAX_VELOCITY = 100.0;
  public static final double WRIST_MAX_MOTION_MAX_ACCELERATION = 100.0;

  public static final MAXMotionConfig WRIST_MAX_MOTION_CONFIG =
      new MAXMotionConfig()
          .allowedClosedLoopError(0.005)
          .maxVelocity(WRIST_MAX_MOTION_MAX_VELOCITY)
          .maxAcceleration(WRIST_MAX_MOTION_MAX_ACCELERATION);

  public static final SparkFlexConfig WRIST_CONFIG =
      (SparkFlexConfig)
          new SparkFlexConfig()
              .inverted(false)
              .idleMode(IdleMode.kBrake)
              .voltageCompensation(RobotController.getBatteryVoltage())
              .smartCurrentLimit(60);

  public static enum WristState {
    PROCESSOR(Rotations.of(0.15)),
    STATION(Rotations.of(0)),
    ALGAE_STOW(Rotations.of(0.30)),
    L1(Rotations.of(0.24)),
    L2_CORAL(Rotations.of(0.18)),
    L2_ALGAE(Rotations.of(0.138)),
    L3_CORAL(Rotations.of(0.185)),
    L3_ALGAE(Rotations.of(0.138)),
    L4(Rotations.of(0.195)),
    NET(Rotations.of(0.35)),
    NEUTRAL(Rotations.of(0));

    public Angle rotations;

    private WristState(Angle rotations) {
      this.rotations = rotations;
    }
  }
}
