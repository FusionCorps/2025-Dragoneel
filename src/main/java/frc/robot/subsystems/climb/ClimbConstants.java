package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class ClimbConstants {
  public static final int CLIMB_MOTOR_ID = 17;
  public static final Voltage CLIMB_RUNOUT_VOLTS = Volts.of(1.0 * 12.0);
  public static final Voltage CLIMB_RETRACT_VOLTS = Volts.of(-0.50 * 12.0);
  public static final Voltage CLIMB_HOLD_VOLTS = Volts.of(-0.06 * 12.0);
}
