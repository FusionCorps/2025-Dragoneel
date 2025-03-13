package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.drive.DriveConstants.AutoAlignDirection.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.AutoAlignDirection;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

public class Autos {
  private Drive drive;
  private Elevator elevator;
  private Wrist wrist;
  private Shooter shooter;
  private ElevatorAndWristCommands elevatorAndWristCommands;

  // end goal: T2-JKLA and B2-EDCB (flipped version of T2-JKLA across the x-axis)
  // All other routines are subsets of these

  // All the paths we use
  private PathPlannerPath CenterStart_H;

  private PathPlannerPath TStart_J;
  private PathPlannerPath J_TCor;
  private PathPlannerPath TCor_K;
  private PathPlannerPath K_TCor;
  private PathPlannerPath TCor_L;
  private PathPlannerPath L_TCor;
  private PathPlannerPath TCor_A;

  private PathPlannerPath BStart_E;
  private PathPlannerPath E_BCor;
  private PathPlannerPath BCor_D;
  private PathPlannerPath D_BCor;
  private PathPlannerPath BCor_C;
  private PathPlannerPath C_BCor;
  private PathPlannerPath BCor_B;

  private PathPlannerPath TOP_PUSH;
  private PathPlannerPath BOTTOM_PUSH;

  private final Time STATION_WAIT_TIME = Seconds.of(0.5);
  private final Time AUTO_ALIGN_TIMEOUT = Seconds.of(0.5);
  private final Time SHOOT_TIMEOUT = Seconds.of(0.75);

  public Autos(Drive drive, Elevator elevator, Wrist wrist, Shooter shooter) {
    this.drive = drive;
    this.elevator = elevator;
    this.wrist = wrist;
    this.shooter = shooter;
    this.elevatorAndWristCommands = new ElevatorAndWristCommands(elevator, wrist);

    // Load all the paths
    try {
      // Center
      CenterStart_H = PathPlannerPath.fromChoreoTrajectory("CenterStart-H");

      // Top
      TStart_J = PathPlannerPath.fromChoreoTrajectory("TStart-J");
      J_TCor = PathPlannerPath.fromChoreoTrajectory("J-TCor");
      TCor_K = PathPlannerPath.fromChoreoTrajectory("TCor-K");
      K_TCor = PathPlannerPath.fromChoreoTrajectory("K-TCor");
      TCor_L = PathPlannerPath.fromChoreoTrajectory("TCor-L");
      L_TCor = PathPlannerPath.fromChoreoTrajectory("L-TCor");
      TCor_A = PathPlannerPath.fromChoreoTrajectory("TCor-A");

      // Bottom
      BStart_E = PathPlannerPath.fromChoreoTrajectory("BStart-E");
      E_BCor = PathPlannerPath.fromChoreoTrajectory("E-BCor");
      BCor_D = PathPlannerPath.fromChoreoTrajectory("BCor-D");
      D_BCor = PathPlannerPath.fromChoreoTrajectory("D-BCor");
      BCor_C = PathPlannerPath.fromChoreoTrajectory("BCor-C");
      C_BCor = PathPlannerPath.fromChoreoTrajectory("C-BCor");
      BCor_B = PathPlannerPath.fromChoreoTrajectory("BCor-B");

      // Pushes
      TOP_PUSH = PathPlannerPath.fromChoreoTrajectory("TopPush");
      BOTTOM_PUSH = PathPlannerPath.fromChoreoTrajectory("BottomPush");

    } catch (Exception e) {
      System.out.println("Failed to load paths. DO NOT RUN AUTO");
      e.printStackTrace();
    }
  }

  // Move straight for 2 seconds
  public Command moveStraight() {
    return DriveCommands.joystickDrive(drive, () -> -0.2, () -> 0, () -> 0).withTimeout(2.0);
  }

  /*
   * General schema for all routines:
   *
   * 1. Reset odometry to starting position
   * 2a. If scoring:
   *   - Follow path
   *   - Align to branch
   *   - Move elevator/wrist and shoot
   * 2b. If picking up from station:
   *  - Move elevator/wrist down to station position
   *  - Follow path
   *  - Wait 0.5 seconds
   * Multi-piece autos alternate between scoring and picking up from station
   */

  public Command onePieceFromCenter() {
    return Commands.sequence(resetOdom(CenterStart_H), moveAndScore(CenterStart_H, RIGHT));
  }

  /* ========== Top autos ========== */
  public Command onePieceFromTop() {
    return Commands.sequence(resetOdom(TStart_J), moveAndScore(TStart_J, RIGHT));
  }

  public Command twoPieceFromTop() {
    return Commands.sequence(onePieceFromTop(), moveAndPickup(J_TCor), moveAndScore(TCor_K, LEFT));
  }

  public Command threePieceFromTop() {
    return Commands.sequence(twoPieceFromTop(), moveAndPickup(K_TCor), moveAndScore(TCor_L, RIGHT));
  }

  public Command fourPieceFromTop() {
    return Commands.sequence(
        threePieceFromTop(), moveAndPickup(L_TCor), moveAndScore(TCor_A, LEFT));
  }

  /* ========== Bottom autos ========== */
  public Command onePieceFromBottom() {
    return Commands.sequence(resetOdom(BStart_E), moveAndScore(BStart_E, LEFT));
  }

  public Command twoPieceFromBottom() {
    return Commands.sequence(
        onePieceFromBottom(), moveAndPickup(E_BCor), moveAndScore(BCor_D, RIGHT));
  }

  public Command threePieceFromBottom() {
    return Commands.sequence(
        twoPieceFromBottom(), moveAndPickup(D_BCor), moveAndScore(BCor_C, LEFT));
  }

  public Command fourPieceFromBottom() {
    return Commands.sequence(
        threePieceFromBottom(), moveAndPickup(C_BCor), moveAndScore(BCor_B, RIGHT));
  }

  public Command doNothing() {
    return Commands.none();
  }

  public Command pushAndOnePieceFromTop() {
    return Commands.sequence(resetOdom(TOP_PUSH), moveAndScore(TOP_PUSH, RIGHT));
  }

  public Command pushAndOnePieceFromBottom() {
    return Commands.sequence(resetOdom(BOTTOM_PUSH), moveAndScore(BOTTOM_PUSH, LEFT));
  }

  /* Helper commands for readability */
  private Command autoAlignAndScore(AutoAlignDirection direction) {
    return Commands.sequence(
        DriveCommands.autoAlignToNearestBranchAuto(drive, direction)
            .withTimeout(AUTO_ALIGN_TIMEOUT),
        elevatorAndWristCommands.goToL4(),
        shooter
            .shootCoralInAutoCmd(
                wrist.isAtScoringState,
                elevator::getCurrentElevatorState,
                RobotContainer.simCoralProjectileSupplier)
            .withTimeout(SHOOT_TIMEOUT));
  }

  private Command resetOdom(PathPlannerPath initialPath) {
    return drive.setPoseCmd(
        new Pose2d(
            initialPath.getPoint(0).position, initialPath.getIdealStartingState().rotation()));
  }

  private Command moveAndScore(PathPlannerPath path, AutoAlignDirection direction) {
    return Commands.sequence(
        resetOdom(path), AutoBuilder.followPath(path), autoAlignAndScore(direction));
  }

  private Command moveAndPickup(PathPlannerPath path) {
    return Commands.sequence(
        elevatorAndWristCommands.goToStation(),
        AutoBuilder.followPath(path),
        Commands.waitTime(STATION_WAIT_TIME));
  }
}
