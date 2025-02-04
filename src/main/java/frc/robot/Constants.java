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
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

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

  public static class ScorerConstants {
    /* Scorer motor ID */
    public static final int SCORER_MOTOR_ID = 0;

    public static final SparkFlexConfig SCORER_CONFIG =
        (SparkFlexConfig)
            new SparkFlexConfig()
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .voltageCompensation(RobotController.getBatteryVoltage())
                .smartCurrentLimit(20);

    /* Scorer motor state */
    public static enum ScorerState {
      IDLE(Volts.of(0.0)),
      OUTTAKE_ALGAE(Volts.of(6.0)),
      SHOOT_CORAL(Volts.of(-6.0));

      public final Voltage volts;

      private ScorerState(Voltage volts) {
        this.volts = volts;
      }
    }
  }

  public static class ClimbConstants {

    public static final int CLIMB_MOTOR_ID = 16;
    public static final Voltage CLIMB_RUN_VOLTS = Volts.of(9.0);
  }

  public static class ElevatorConstants {

    public static enum ElevatorState {
      STOW(Rotations.of(0.0)),
      L1(Rotations.of(0.5)),
      L2(Rotations.of(0.8)),
      STATION(Rotations.of(0.85)),
      L3(Rotations.of(1.1)),
      L4(Rotations.of(1.8)),
      NET(Rotations.of(2.2));

      public final Angle height;

      private ElevatorState(Angle height) {
        this.height = height;
      }
    }

    public static final int mainElevatorMotorID = 13;
    public static final int followerElevatorMotorID = 14;

    public static final double elevatorGearRatio = 60.0 / 14.0;
    public static final double elevatorShaftRadiusInches = 0.5 / 2.0; // 1/2" thunderhex shaft
  }
}
