package frc.robot.subsystems.wrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    double WristVoltage = 0.0;
    double WristAmps = 0.0;
    double wristVelocityRadPerSec = 0.0;
    double WristPositionRad = 0.0;
    boolean WristMotorConnected = false;
  }

  default void updateInputs(WristIOInputs inputs) {}

  default void setVoltage(Voltage voltage) {}

  default void setTargetPosition(Angle angle) {}
}
