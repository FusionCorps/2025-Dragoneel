package frc.robot.subsystems.wrist;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ScoringPieceType;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    boolean connected = false;
    double positionRad = 0.0;
    double absolutePositionRad = 0.0;
    double velocityRadPerSec = 0.0;
    double appliedVolts = 0.0;
    double currentAmps = 0.0;

    double wristSetpointRad = 0.0;
  }

  default void updateInputs(WristIOInputs inputs) {}

  default void setVoltageOpenLoop(Voltage voltage) {}

  default void setTargetPosition(Angle angle, Supplier<ScoringPieceType> scoringModeType) {}

  default void setToCoralSpeed() {}

  default void setToAlgaeSpeed() {}

  default void toggleSpeed() {}
}
