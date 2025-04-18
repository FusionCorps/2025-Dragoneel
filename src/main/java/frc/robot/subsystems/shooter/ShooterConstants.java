package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.LoggedTunableNumber;

public class ShooterConstants {
  /* Scorer motor ID */
  public static final int SHOOTER_MOTOR_ID = 16;

  public static final SparkFlexConfig SHOOTER_CONFIG =
      (SparkFlexConfig)
          new SparkFlexConfig()
              .inverted(false)
              .idleMode(IdleMode.kBrake)
              .voltageCompensation(RobotController.getBatteryVoltage())
              .smartCurrentLimit(180);

  /* Scorer motor state */
  public static enum ShooterState {
    IDLE(Volts.of(0.0)),
    SHOOT_ALGAE(Volts.of(-0.6 * 12.0)),
    SHOOT_CORAL_DEFAULT(Volts.of(0.15 * 12.0)),
    SHOOT_CORAL_L4(Volts.of(0.13 * 12.0)),
    PULL_IN_ALGAE(Volts.of(0.40 * 12.0)),
    SHOOT_CORAL_L1(Volts.of(0.13 * 12.0));

    public LoggedTunableNumber volts;

    private ShooterState(Voltage volts) {
      this.volts = new LoggedTunableNumber("/Shooter/" + this.name(), volts.in(Volts));
    }
  }
}
