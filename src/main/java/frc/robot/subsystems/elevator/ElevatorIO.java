package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    boolean mainConnected = false;
    double mainPositionRad = 0.0;
    double mainVelocityRadPerSec = 0.0;
    double mainAppliedVolts = 0.0;
    double mainCurrentAmps = 0.0;

    boolean followerConnected = false;
    double followerPositionRad = 0.0;
    double followerVelocityRadPerSec = 0.0;
    double followerAppliedVolts = 0.0;
    double followerCurrentAmps = 0.0;

    double positionSetpointRad = 0.0;

    // boolean reverseLimitSwitchTriggered = false;
    // boolean forwardLimitSwitchTriggered = false;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setTargetPosition(Angle motorTargetRotations) {}

  default void setVoltageOpenLoop(Voltage volts) {}

  /** Only necessary in use with {@link #setVoltageOpenLoop()} */
  default void holdPosition() {}

  default void zeroPosition() {}

  default void changemotionmagic() {}
}
