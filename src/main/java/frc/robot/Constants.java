// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class ElevatorConstants {

    public static enum ElevatorState {
      STOW(Rotations.of(0.0)),
      L1(Rotations.of(3.0)),
      L2(Rotations.of(6.0)),
      L3(Rotations.of(9.0)),
      L4(Rotations.of(12.0)),
      NET(Rotations.of(15.0));

      public final Angle height;

      private ElevatorState(Angle height) {
        this.height = height;
      }
    }

    public static final int mainElevatorMotorID = 13;
    public static final int followerElevatorMotorID = 14;

    public static final double elevatorGearRatio = 6.0;
    public static final double elevatorShaftRadiusInches = 0.5;

    public static final TalonFXConfiguration elevatorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(80)
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(70)
                    .withSupplyCurrentLowerLimit(40)
                    .withSupplyCurrentLowerTime(1.0))
            .withSlot0(
                new Slot0Configs()
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    // TODO: these need to be tuned
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKS(0)
                    .withKV(0)
                    .withKA(0))
            .withMotionMagic(
                new MotionMagicConfigs()
                    // TODO: these need to change
                    .withMotionMagicCruiseVelocity(0)
                    .withMotionMagicAcceleration(0)
                    .withMotionMagicJerk(0))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    // TODO: these may need to change
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(100)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(0));
  }
}
