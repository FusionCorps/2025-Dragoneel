package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    boolean connected = false;
    double positionRad = 0.0;
    double velocityRadPerSec = 0.0;
    double appliedVolts = 0.0;
    double currentAmps = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setVoltage(Voltage voltage) {}
}
