package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.drive.DriveConstants.AutoAlignDirection.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.AutoAlignDirection;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.Wrist;

public class Autos {
  private Drive drive;
  private Shooter shooter;
  private ElevatorAndWristCommands elevatorAndWristCommands;

  // routines: TStart-JLKA and BStart-ECDB, LeftPush and RightPush, CenterStart-H, CenterAlgae-Barge
  // All routines are subsets of these

  // the paths we use
  private PathPlannerPath CenterStart_H;

  private PathPlannerPath TStart_J;
  private PathPlannerPath J_TCor;
  private PathPlannerPath K_TCor;
  private PathPlannerPath L_TCor;
  private PathPlannerPath TCor_A;

  private PathPlannerPath BStart_E;
  private PathPlannerPath E_BCor;
  private PathPlannerPath D_BCor;
  private PathPlannerPath C_BCor;
  private PathPlannerPath BCor_B;

  private PathPlannerPath LEFT_PUSH;
  private PathPlannerPath RIGHT_PUSH;
  private PathPlannerPath CenterAlgae_Barge;

  private final Time STATION_WAIT_TIME = Seconds.of(0.7);
  private final Time AUTO_ALIGN_TIMEOUT = Seconds.of(4.0);
  private final Time SHOOT_TIMEOUT = Seconds.of(1.1);

  public Autos(Drive drive, Elevator elevator, Wrist wrist, Shooter shooter) {
    this.drive = drive;
    this.shooter = shooter;
    this.elevatorAndWristCommands = new ElevatorAndWristCommands(elevator, wrist);

    // Load all the paths
    try {
      // Center
      CenterStart_H = PathPlannerPath.fromChoreoTrajectory("CenterStart-H");

      // Top
      TStart_J = PathPlannerPath.fromChoreoTrajectory("TStart-J");
      J_TCor = PathPlannerPath.fromChoreoTrajectory("J-TCor");
      K_TCor = PathPlannerPath.fromChoreoTrajectory("K-TCor");
      L_TCor = PathPlannerPath.fromChoreoTrajectory("L-TCor");
      TCor_A = PathPlannerPath.fromChoreoTrajectory("TCor-A");

      // Bottom
      BStart_E = PathPlannerPath.fromChoreoTrajectory("BStart-E");
      E_BCor = PathPlannerPath.fromChoreoTrajectory("E-BCor");
      D_BCor = PathPlannerPath.fromChoreoTrajectory("D-BCor");
      C_BCor = PathPlannerPath.fromChoreoTrajectory("C-BCor");
      BCor_B = PathPlannerPath.fromChoreoTrajectory("BCor-B");

      // Pushes
      LEFT_PUSH = PathPlannerPath.fromChoreoTrajectory("LeftPush");
      RIGHT_PUSH = PathPlannerPath.fromChoreoTrajectory("RightPush");
      CenterAlgae_Barge = PathPlannerPath.fromChoreoTrajectory("CenterAlgae-Barge");

    } catch (Exception e) {
      System.out.println("Failed to load paths. DO NOT RUN AUTO");
      e.printStackTrace();
    }
  }

  // Move straight for 2 seconds
  public Command moveStraight() {
    return DriveCommands.joystickDrive(drive, () -> 0.5, () -> 0, () -> 0).withTimeout(2.0);
  }

  /*
   * General schema for all routines:
   *
   * 1. Reset odometry to starting position
   * 2a. If scoring:
   *   - Auto-Align to branch (works at a distance)
   *   - Move elevator/wrist and shoot
   * 2b. If picking up from station:
   *  - Move elevator/wrist down to station position
   *  - Follow path
   *  - Wait 0.5 seconds
   * Multi-piece autos alternate between scoring and picking up from station
   */

  public Command onePieceFromCenter() {
    return Commands.sequence(resetOdometry(CenterStart_H), autoAlignAndScore(RIGHT));
  }

  public Command onePieceFromCenterBlind() {
    return Commands.sequence(
        Commands.runOnce(() -> Vision.blind = true),
        resetOdometry(CenterStart_H),
        driveBlindAndScore());
  }

  /* ========== Top autos JLKA ========== */
  public Command onePieceFromLeft() {
    return Commands.sequence(resetOdometry(TStart_J), autoAlignAndScore(RIGHT));
  }

  public Command twoPieceFromLeft() {
    return Commands.sequence(
        onePieceFromLeft(), moveToStationAndPickup(J_TCor), autoAlignAndScore(RIGHT));
  }

  public Command threePieceFromLeft() {
    return Commands.sequence(
        twoPieceFromLeft(), moveToStationAndPickup(L_TCor), autoAlignAndScore(LEFT));
  }

  public Command fourPieceFromLeft() {
    return Commands.sequence(
        threePieceFromLeft(),
        moveToStationAndPickup(K_TCor),
        AutoBuilder.followPath(TCor_A),
        autoAlignAndScore(LEFT));
  }

  /* ========== Bottom autos ECDB ========== */
  public Command onePieceFromRight() {
    return Commands.sequence(resetOdometry(BStart_E), autoAlignAndScore(LEFT));
  }

  public Command twoPieceFromRight() {
    return Commands.sequence(
        onePieceFromRight(), moveToStationAndPickup(E_BCor), autoAlignAndScore(LEFT));
  }

  public Command threePieceFromRight() {
    return Commands.sequence(
        twoPieceFromRight(), moveToStationAndPickup(C_BCor), autoAlignAndScore(RIGHT));
  }

  public Command fourPieceFromRight() {
    return Commands.sequence(
        threePieceFromRight(),
        moveToStationAndPickup(D_BCor),
        AutoBuilder.followPath(BCor_B),
        autoAlignAndScore(RIGHT));
  }

  public Command doNothing() {
    return Commands.none();
  }

  // never tested
  public Command pushAndOnePieceFromLeft() {
    return Commands.sequence(
        resetOdometry(LEFT_PUSH), AutoBuilder.followPath(LEFT_PUSH), autoAlignAndScore(RIGHT));
  }

  // never tested
  public Command pushAndOnePieceFromRight() {
    return Commands.sequence(
        resetOdometry(RIGHT_PUSH), AutoBuilder.followPath(RIGHT_PUSH), autoAlignAndScore(LEFT));
  }

  // never tested
  public Command onePieceCenterWithAlgae() {
    return Commands.sequence(
        onePieceFromCenter(),
        DriveCommands.joystickDrive(drive, () -> -0.5, () -> 0, () -> 0).withTimeout(2.0),
        autoAlignAndScore(ALGAE),
        AutoBuilder.followPath(CenterAlgae_Barge),
        shooter.shootAlgaeCmd().withTimeout(3.0));
  }

  private Command driveBlindAndScore() {
    return Commands.sequence(
        AutoBuilder.followPath(CenterStart_H),
        elevatorAndWristCommands.goToL4(),
        shooter.pulseShooterAutoCmd().withTimeout(SHOOT_TIMEOUT),
        elevatorAndWristCommands.goToStation());
  }

  /* Helper commands for readability */
  private Command autoAlignAndScore(AutoAlignDirection direction) {
    return Commands.parallel(
            DriveCommands.autoAlignToNearest(drive, direction).withTimeout(AUTO_ALIGN_TIMEOUT),
            Commands.waitUntil(
                    () ->
                        drive
                                .getPose()
                                .getTranslation()
                                .getDistance(DriveCommands.autoAlignTarget.getTranslation())
                            < 1.5)
                .andThen(elevatorAndWristCommands.goToL4()))
        .andThen(shooter.pulseShooterAutoCmd().withTimeout(SHOOT_TIMEOUT));
  }

  private Command resetOdometry(PathPlannerPath initialPath) {
    return drive.setPoseCmd(
        new Pose2d(
            initialPath.getPoint(0).position, initialPath.getIdealStartingState().rotation()));
  }

  private Command moveToStationAndPickup(PathPlannerPath path) {
    return Commands.parallel(elevatorAndWristCommands.goToStation(), AutoBuilder.followPath(path))
        .andThen(Commands.waitTime(STATION_WAIT_TIME));
  }
}
