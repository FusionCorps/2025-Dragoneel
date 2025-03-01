package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;

public class ShooterConstants {
  /* Scorer motor ID */
  public static final int SHOOTER_MOTOR_ID = 16;

  public static final SparkFlexConfig SHOOTER_CONFIG =
      (SparkFlexConfig)
          new SparkFlexConfig()
              .inverted(true)
              .idleMode(IdleMode.kBrake)
              .voltageCompensation(RobotController.getBatteryVoltage())
              .smartCurrentLimit(120);

  /* Scorer motor state */
  public static enum ShooterState {
    IDLE(Volts.of(0.0)),
    SHOOT_ALGAE(Volts.of(0.35 * 12.0)),
    SHOOT_CORAL_DEFAULT(Volts.of(0.20 * 12.0)),
    SHOOT_CORAL_L4(Volts.of(0.165 * 12.0)),
    SHOOT_CORAL_L1(Volts.of(0.15 * 12.0));

    public final Voltage volts;

    private ShooterState(Voltage volts) {
      this.volts = volts;
    }
  }
}
