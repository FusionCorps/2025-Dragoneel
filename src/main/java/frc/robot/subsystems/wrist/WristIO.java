package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    boolean connected = false;
    double positionRad = 0.0;
    Rotation2d absolutePositionRad = new Rotation2d();
    double velocityRadPerSec = 0.0;
    double appliedVolts = 0.0;
    double currentAmps = 0.0;

    Rotation2d wristSetpoint = new Rotation2d();
  }

  default void updateInputs(WristIOInputs inputs) {}

  default void setVoltageOpenLoop(Voltage voltage) {}

  default void setTargetPosition(Angle angle) {}
}
