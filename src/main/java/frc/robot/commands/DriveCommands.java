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

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double AUTO_DRIVE_MAX_SPEED = SPEED_AT_12V.in(MetersPerSecond);
  private static final double AUTO_DRIVE_MAX_ROT = Units.rotationsToRadians(0.75);
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 1.0;
  private static final double ANGLE_KD = 0.0;
  private static final double ANGLE_MAX_VELOCITY = Units.rotationsToRadians(1.25);
  private static final double ANGLE_MAX_ACCELERATION = Units.rotationsToRadians(1.0);
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  public static DriveSpeedMode speedMode = DriveSpeedMode.DEFAULT;

  @AutoLogOutput(key = "AutoAlign/targetAutoAlignPose")
  public static Pose2d autoAlignTarget = new Pose2d();

  @AutoLogOutput(key = "AutoAlign/autoAlignDirection")
  public static AutoAlignDirection autoAlignDirection = AutoAlignDirection.LEFT;

  public static boolean isAutoAligning = false;

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.driveRobotCentric(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /** Field relative drive command using PID for full control to a specified pose. */
  private static Command driveToPose(Drive drive, Supplier<Pose2d> poseSupplier) {
    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(Units.degreesToRadians(1.0));

    PIDController xController = new PIDController(0.8, 0.0, 0.0);
    xController.setTolerance(0.02);
    PIDController yController = new PIDController(0.8, 0.0, 0.0);
    yController.setTolerance(0.02);

    // Construct command
    return new FunctionalCommand(
        () -> {
          // Reset PID controllerS when command starts
          angleController.reset(drive.getRotation().getRadians());
          xController.reset();
          yController.reset();
        },
        () -> {
          // calculate linear velocities
          double xVel = xController.calculate(drive.getPose().getX(), poseSupplier.get().getX());
          double yVel = yController.calculate(drive.getPose().getY(), poseSupplier.get().getY());

          // Calculate angular speed
          double omega =
              angleController.calculate(
                  drive.getRotation().getRadians(), poseSupplier.get().getRotation().getRadians());

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  xVel * AUTO_DRIVE_MAX_SPEED,
                  yVel * AUTO_DRIVE_MAX_SPEED,
                  omega * AUTO_DRIVE_MAX_ROT);
          drive.driveRobotCentric(
              ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
        },
        interrupted -> {
          drive.driveRobotCentric(new ChassisSpeeds());
          xController.close();
          yController.close();
        },
        () -> {
          return xController.atSetpoint()
              && yController.atSetpoint()
              && angleController.atSetpoint();
        }, // end automatically when at setpoint
        drive);
  }

  /**
   * Field relative drive command using PID for full control, targeting the currently seen reef tag.
   * This will drive to ??? m in front of the tag.
   *
   * @param drive The drive subsystem
   * @param tagPoseSupplier A supplier for the tag pose - should be the actual tag pose, not the
   *     pose of the robot relative to the tag or any offset pose.
   */
  private static Command autoAlignTo(
      Drive drive,
      Supplier<Pose3d> tagPoseSupplierNoOffset,
      AutoAlignDirection autoAlignDirection) {
    // applies x/y offsets from the tag pose
    // rotate, because apriltag will always be 180° from robot
    Supplier<Pose2d> tagPoseSupplierIn2DWOffset =
        () -> {
          if (tagPoseSupplierNoOffset.get() == null) {
            return drive.getPose();
          } else {
            double outOffset;
            double sideOffset;
            switch (autoAlignDirection) {
              case LEFT:
                outOffset = autoAlignOutCoralLeft.get() * 0.01 + autoAlignOutCoralLeftBaseline;
                sideOffset = autoAlignSideCoralLeft.get() * 0.01 + autoAlignSideCoralLeftBaseline;
                break;
              case RIGHT:
                outOffset = autoAlignOutCoralRight.get() * 0.01 + autoAlignOutCoralRightBaseline;
                sideOffset = autoAlignSideCoralRight.get() * 0.01 + autoAlignSideCoralRightBaseline;
                break;
              case ALGAE:
                outOffset = autoAlignOutAlgae.get() * 0.01 + autoAlignOutAlgaeBaseline;
                sideOffset = autoAlignSideAlgae.get() * 0.01 + autoAlignSideAlgaeBaseline;
                break;
              case BARGE:
                outOffset = autoAlignOutBarge.get() * 0.01 + autoAlignOutBargeBaseline;
                sideOffset = autoAlignSideBarge.get() * 0.01 + autoAlignSideBargeBaseline;
                break;
              default:
                outOffset = 0;
                sideOffset = 0;
            }

            return tagPoseSupplierNoOffset
                .get()
                .transformBy(
                    new Transform3d(outOffset, sideOffset, 0, new Rotation3d(Rotation2d.kPi)))
                .toPose2d();
          }
        };

    return driveToPose(drive, tagPoseSupplierIn2DWOffset)
        .deadlineFor(
            Commands.run(
                () -> {
                  Logger.recordOutput(
                      "AutoAlign/targetAutoAlignPose", tagPoseSupplierIn2DWOffset.get());
                  Logger.recordOutput("AutoAlign/autoAlignDirection", autoAlignDirection);
                  DriveCommands.autoAlignTarget = tagPoseSupplierIn2DWOffset.get();
                  DriveCommands.autoAlignDirection = autoAlignDirection;
                  isAutoAligning = true;
                }))
        .finallyDo(() -> isAutoAligning = false);
  }

  /**
   * Field relative drive command using PID for full control, targeting the nearest reef tag
   * (doesn't have to be seen). Separate commands should be created for aligning to left and right
   * branches.
   */
  public static Command autoAlignToNearest(Drive drive, AutoAlignDirection autoAlignDirection) {
    return autoAlignTo(
        drive,
        () ->
            new Pose3d(
                drive
                    .getPose()
                    .nearest(
                        DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get() == Alliance.Red
                            ? redReefTagPoses
                            : blueReefTagPoses)),
        autoAlignDirection);
  }

  public static Command autoAlignToBarge(Drive drive) {
    return autoAlignTo(
        drive,
        () ->
            new Pose3d(
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red
                    ? aprilTagLayout.getTagPose(5).get().toPose2d()
                    : aprilTagLayout.getTagPose(14).get().toPose2d()),
        AutoAlignDirection.BARGE);
  }
  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return drive.startRun(
        // Reset PID controller when command starts
        () -> angleController.reset(drive.getRotation().getRadians()),
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Calculate angular speed
          double omega =
              angleController.calculate(
                  drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

          // Convert field relative speeds to internal robot relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.driveRobotCentric(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        });
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Targets the currently seen reef apriltag and rotates to the wall that it's located on.
   */
  public static Command rotateToReefTagWall(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Pose3d> tagPoseSupplier) {
    return joystickDriveAtAngle(
        drive,
        xSupplier,
        ySupplier,
        () -> {
          if (tagPoseSupplier.get() == null) {
            return drive.getRotation(); // if no tag, maintain current robot rotation
          } else {
            return tagPoseSupplier.get().toPose2d().getRotation().rotateBy(Rotation2d.kPi);
          }
        });
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.driveRobotCentric(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
