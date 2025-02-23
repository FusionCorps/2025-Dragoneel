package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;

public class WristConstants {
  public static final int WRIST_MOTOR_ID = 15;

  // public static final double WRIST_kG = 0.224;
  public static final SparkFlexConfig WRIST_CONFIG =
      (SparkFlexConfig)
          new SparkFlexConfig()
              .inverted(false)
              .idleMode(IdleMode.kBrake)
              .voltageCompensation(RobotController.getBatteryVoltage())
              .smartCurrentLimit(60)
              .apply(new AbsoluteEncoderConfig().zeroCentered(true))
              .apply(
                  new ClosedLoopConfig()
                      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                      .positionWrappingInputRange(-0.5, 0.5)
                      .positionWrappingEnabled(true)
                      .pid(2.0, 0, 0))
      // .apply(
      //     new SoftLimitConfig()
      //         .reverseSoftLimitEnabled(true)
      //         .reverseSoftLimit(0)
      //         .forwardSoftLimitEnabled(true)
      //         .forwardSoftLimit(0.35)
      ;

  public static enum WristState {
    ZERO(Rotations.zero()),
    PROCESSOR(Rotations.of(0.15)),
    STATION(Rotations.of(0)),
    L1(Rotations.of(0.24)),
    L2_AND_L3(Rotations.of(0.13)),
    L4(Rotations.of(0.215)),
    NET(Rotations.of(0.255));

    public Angle rotations;

    private WristState(Angle rotations) {
      this.rotations = rotations;
    }
  }
}
