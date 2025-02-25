package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class ClimbConstants {
  public static final int CLIMB_MOTOR_ID = 17;
  public static final Voltage CLIMB_RUN_VOLTS =
      Volts.of(
          0.2 * 12.0); // positive output will move climber in to elevate, negative moves out to
  // lower robot
}
