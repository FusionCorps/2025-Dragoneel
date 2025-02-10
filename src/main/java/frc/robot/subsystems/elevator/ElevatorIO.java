package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    boolean mainElevatorMotorConnected = false;
    double mainElevatorPositionRad = 0.0;
    double mainElevatorVelocityRadPerSec = 0.0;
    double mainElevatorAppliedVolts = 0.0;
    double mainElevatorCurrentAmps = 0.0;

    boolean followerElevatorMotorConnected = false;
    double followerElevatorPositionRad = 0.0;
    double followerElevatorVelocityRadPerSec = 0.0;
    double followerElevatorAppliedVolts = 0.0;
    double followerElevatorCurrentAmps = 0.0;

    double elevatorPositionSetpointRad = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setTargetPosition(Angle motorTargetRotations) {}

  default void setVoltageOpenLoop(Voltage volts) {}

  default void zeroPosition() {}
}
