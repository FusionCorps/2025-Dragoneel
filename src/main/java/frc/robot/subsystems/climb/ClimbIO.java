package frc.robot.subsystems.climb;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    boolean climbMotorConnected = false;
    double climbPositionRad = 0.0;
    double climbVelocityRadPerSec = 0.0;
    double climbAppliedVolts = 0.0;
    double climbCurrentAmps = 0.0;
  }

  default void updateInputs(ClimbIOInputs inputs) {}

  default void setVoltage(Voltage voltage) {}

  default void holdPosition() {}
}
