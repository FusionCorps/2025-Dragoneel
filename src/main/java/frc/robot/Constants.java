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

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.wrist.WristConstants.WristState;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode SIM_MODE = Mode.SIM;
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum TargetState {
    STATION(ElevatorState.STATION, WristState.STATION),
    PROCESSOR(ElevatorState.PROCESSOR, WristState.PROCESSOR),
    ALGAE_STOW(ElevatorState.ALGAE_STOW, WristState.ALGAE_STOW),
    L1(ElevatorState.L1, WristState.L1),
    L2_CORAL(ElevatorState.L2, WristState.L2_CORAL),
    L2_ALGAE(ElevatorState.L2, WristState.L2_ALGAE),
    L3_CORAL(ElevatorState.L3, WristState.L3_CORAL),
    L3_ALGAE(ElevatorState.L3, WristState.L3_ALGAE),
    L4(ElevatorState.L4, WristState.L4),
    NET(ElevatorState.NET, WristState.NET),
    NEUTRAL(ElevatorState.NEUTRAL, WristState.NEUTRAL);

    public final ElevatorState elevatorState;
    public final WristState wristState;

    private TargetState(ElevatorState elevatorState, WristState wristState) {
      this.elevatorState = elevatorState;
      this.wristState = wristState;
    }
  }

  public static enum ScoringPieceType {
    CORAL,
    ALGAE
  }

  // TODO: Update these values in PathPlanner and Choreo and here
  public static final Mass ROBOT_MASS = Pounds.of(111.4 + 17.8 + 15.0);
  public static final double ROBOT_MOI =
      //   ROBOT_MASS.in(Kilograms) * DriveConstants.FRONT_LEFT.LocationX * (0.011992 /
      // DriveConstants.driveGains.kA);
      5.0; // round number on the higher/safer side, higher MOI means slower path rotation which is
  // OK
  // default PathPlanner configuration for path following
  public static final RobotConfig PP_ROBOT_CONFIG_DEFAULT =
      new RobotConfig(
          Constants.ROBOT_MASS.in(Kilograms),
          Constants.ROBOT_MOI,
          new ModuleConfig(
              DriveConstants.FRONT_LEFT.WheelRadius,
              DriveConstants.SPEED_AT_12V.in(MetersPerSecond),
              DriveConstants.WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(DriveConstants.FRONT_LEFT.DriveMotorGearRatio),
              DriveConstants.FRONT_LEFT.SlipCurrent,
              1),
          DriveConstants.MODULE_TRANSLATIONS);
}
