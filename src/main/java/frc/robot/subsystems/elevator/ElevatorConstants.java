package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
  public static final int MAIN_ELEVATOR_MOTOR_ID = 13;
  public static final int FOLLOWER_ELEVATOR_MOTOR_ID = 14;

  public static enum ElevatorState {
    ZERO(Rotations.of(0.0)),
    PROCESSOR(Rotations.of(0.0)),
    L1(Rotations.of(1)),
    L2(Rotations.of(2)),
    STATION(Rotations.of(1)),
    L3(Rotations.of(10)),
    L4(Rotations.of(15)),
    NET(Rotations.of(20));

    public Angle rotations;

    private ElevatorState(Angle rotations) {
      this.rotations = rotations;
    }
  }

  public static final double ELEVATOR_kP = 0.4;
  public static final double ELEVATOR_kI = 0.0;
  public static final double ELEVATOR_kD = 0.0;
  public static final double ELEVATOR_kS = 0.0;
  public static final double ELEVATOR_kV = 0.125;
  public static final double ELEVATOR_kG = 0.35;
  public static final double ELEVATOR_kA = 0.0;

  public static final double ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY = 200;
  public static final double ELEVATOR_MOTION_MAGIC_ACCELERATION = 200;

  public static final CurrentLimitsConfigs ELEVATOR_CURRENT_LIMITS_CONFIGS =
      new CurrentLimitsConfigs()
          .withStatorCurrentLimitEnable(true)
          .withStatorCurrentLimit(80)
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(70)
          .withSupplyCurrentLowerLimit(40)
          .withSupplyCurrentLowerTime(1.0);

  public static final SoftwareLimitSwitchConfigs ELEVATOR_SOFT_LIMITS_CONFIGS =
      new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(Rotations.of(26.260565610162730401865820956465))
          .withReverseSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(Rotations.of(0));

  public static final double ELEVATOR_GEAR_RATIO =
      60.0 / 14.0; // 14 shaft rotations for 60 motor rotations
  public static final Distance ELEVATOR_SPOOL_DIAMETER = Inches.of(1.0); // 1" spool
}
