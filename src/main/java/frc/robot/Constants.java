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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.DriveConstants;

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

  public static final double ROBOT_MASS_KG = 74.088;
  public static final double ROBOT_MOI =
      ROBOT_MASS_KG * Units.inchesToMeters(20.75) / 2.0 * 0.011992 / (DriveConstants.driveGains.kA);
  public static final RobotConfig DEFAULT_PP_ROBOT_CONFIG =
      new RobotConfig(
          Constants.ROBOT_MASS_KG,
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
