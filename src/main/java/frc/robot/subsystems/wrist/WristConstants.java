package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;

public class WristConstants {
  public static final int WRIST_MOTOR_ID = 15;

  public static final SparkFlexConfig WRIST_CONFIG =
      (SparkFlexConfig)
          new SparkFlexConfig()
              .inverted(false)
              .idleMode(IdleMode.kBrake)
              .voltageCompensation(RobotController.getBatteryVoltage())
              .smartCurrentLimit(60)
              .apply(
                  new ClosedLoopConfig()
                      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                      .pid(0, 0, 0));

  public static enum WristState {
    ZERO(Rotations.zero()),
    PROCESSOR(Rotations.of(0.15)),
    L1(Rotations.of(0.2)),
    L2_AND_L3(Rotations.of(0.16)),
    L4(Rotations.of(0.16)),
    STATION(Rotations.of(0.154)),
    NET(Rotations.of(0.2));

    public Angle rotations;

    private WristState(Angle rotations) {
      this.rotations = rotations;
    }
  }
}
