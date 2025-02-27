package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

public class ShootingUtil {
  public static ReefscapeCoralOnFly createCoralProjectile(
      Drive drive, Elevator elevator, Wrist wrist, Shooter scorer) {
    return new ReefscapeCoralOnFly(
        drive.getPose().getTranslation(),
        new Translation2d(0.5, -0.1),
        drive.getChassisSpeeds(),
        drive.getRotation(),
        Robot.componentPoses[2].getMeasureZ().minus(Meters.of(0.05)),
        MetersPerSecond.of(1.0),
        wrist.getCurrentAngle().minus(Degrees.of(90.0)));
  }
}
