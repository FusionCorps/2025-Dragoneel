package frc.robot.subsystems.scorer;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ScorerConstants.ScorerState;

import org.littletonrobotics.junction.AutoLog;

public interface ScorerIO {
  @AutoLog
  public static class ScorerIOInputs {
    boolean scorerMotorConnected = false;
    double scorerPositionRad = 0.0;
    double scorerVelocityRadPerSec = 0.0;
    double scorerAppliedVolts = 0.0;
    double scorerCurrentAmps = 0.0;
  }

  default void updateInputs(ScorerIOInputs inputs) {}

  default void setVoltage(Voltage voltage) {}
}
