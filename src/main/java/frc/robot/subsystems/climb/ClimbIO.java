package frc.robot.subsystems.climb;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    boolean connected = false;
    double positionRad = 0.0;
    double velocityRadPerSec = 0.0;
    double appliedVolts = 0.0;
    double currentAmps = 0.0;
  }

  default void updateInputs(ClimbIOInputs inputs) {}

  default void setVoltage(Voltage voltage) {}
}
