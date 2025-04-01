package frc.robot.subsystems.climb;

import frc.robot.util.LoggedTunableNumber;

public class ClimbConstants {
  public static final int CLIMB_MOTOR_ID = 17;
  public static final LoggedTunableNumber CLIMB_RUNOUT_PCT =
      new LoggedTunableNumber("/Climb/CLIMB_RUNOUT_VOLTS", 1.0);
  public static final LoggedTunableNumber CLIMB_RETRACT_PCT =
      new LoggedTunableNumber("/Climb/CLIMB_RETRACT_VOLTS", -1.0);
  public static final LoggedTunableNumber CLIMB_HOLD_PCT =
      new LoggedTunableNumber("/Climb/CLIMB_HOLD_VOLTS", -0.03);
  public static final LoggedTunableNumber holdPosition = new LoggedTunableNumber("holdPos", 145);
  public static final LoggedTunableNumber bottomPos = new LoggedTunableNumber("bottomPos", 515);
}
