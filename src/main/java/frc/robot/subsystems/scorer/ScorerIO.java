package frc.robot.subsystems.scorer;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ScorerIO {
  @AutoLog
  public static class ScorerIOInputs {
    boolean connected = false;
    double positionRad = 0.0;
    double velocityRadPerSec = 0.0;
    double appliedVolts = 0.0;
    double currentAmps = 0.0;
  }

  default void updateInputs(ScorerIOInputs inputs) {}

  default void setVoltage(Voltage voltage) {}
}
