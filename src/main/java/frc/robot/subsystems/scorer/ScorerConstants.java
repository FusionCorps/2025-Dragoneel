package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;

public class ScorerConstants {
  /* Scorer motor ID */
  public static final int SCORER_MOTOR_ID = 16;

  public static final SparkFlexConfig SCORER_CONFIG =
      (SparkFlexConfig)
          new SparkFlexConfig()
              .inverted(true)
              .idleMode(IdleMode.kBrake)
              .voltageCompensation(RobotController.getBatteryVoltage())
              .smartCurrentLimit(120);

  /* Scorer motor state */
  public static enum ScorerState {
    IDLE(Volts.of(0.0)),
    SHOOT_ALGAE(Volts.of(-0.25 * 12.0)),
    SHOOT_CORAL_DEFAULT(Volts.of(0.40 * 12.0)),
    SHOOT_CORAL_L1(Volts.of(0.15 * 12.0));

    public final Voltage volts;

    private ScorerState(Voltage volts) {
      this.volts = volts;
    }
  }
}
