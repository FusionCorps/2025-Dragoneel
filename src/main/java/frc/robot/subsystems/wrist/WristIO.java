package frc.robot.subsystems.wrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    boolean wristMotorConnected = false;
    double wristPositionRad = 0.0;
    double wristVelocityRadPerSec = 0.0;
    double wristAppliedVolts = 0.0;
    double wristCurrentAmps = 0.0;

    double wristSetpointRad = 0.0;
  }

  default void updateInputs(WristIOInputs inputs) {}

  default void setVoltageOpenLoop(Voltage voltage) {}

  default void setTargetPosition(Angle angle) {}
}
