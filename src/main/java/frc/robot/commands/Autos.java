package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.drive.DriveConstants.AutoAlignDirection.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private PathPlannerPath C4_H;

  private PathPlannerPath T2_J;
  private PathPlannerPath J_TStat;
  private PathPlannerPath TStat_K;
  private PathPlannerPath K_TStat;
  private PathPlannerPath TStat_L;
  private PathPlannerPath L_TStat;
  private PathPlannerPath TStat_A;

  private PathPlannerPath B2_E;
  private PathPlannerPath E_BStat;
  private PathPlannerPath BStat_D;
  private PathPlannerPath D_BStat;
  private PathPlannerPath BStat_C;
  private PathPlannerPath C_BStat;
  private PathPlannerPath BStat_B;

  private PathPlannerPath TOP_PUSH;

  public Autos(Drive drive, Elevator elevator, Wrist wrist, Shooter shooter) {
    this.drive = drive;
    this.elevator = elevator;
    this.wrist = wrist;
    this.shooter = shooter;
    this.elevatorAndWristCommands = new ElevatorAndWristCommands(elevator, wrist);

    // Load all the paths
    try {
      C4_H = PathPlannerPath.fromChoreoTrajectory("C4-H");

      T2_J = PathPlannerPath.fromChoreoTrajectory("T2-J");
      J_TStat = PathPlannerPath.fromChoreoTrajectory("J-TStat");
      TStat_K = PathPlannerPath.fromChoreoTrajectory("TStat-K");
      K_TStat = PathPlannerPath.fromChoreoTrajectory("K-TStat");
      TStat_L = PathPlannerPath.fromChoreoTrajectory("TStat-L");
      L_TStat = PathPlannerPath.fromChoreoTrajectory("L-TStat");
      TStat_A = PathPlannerPath.fromChoreoTrajectory("TStat-A");

      B2_E = PathPlannerPath.fromChoreoTrajectory("T2-J").mirrorPath();
      E_BStat = PathPlannerPath.fromChoreoTrajectory("J-TStat").mirrorPath();
      BStat_D = PathPlannerPath.fromChoreoTrajectory("TStat-K").mirrorPath();
      D_BStat = PathPlannerPath.fromChoreoTrajectory("K-TStat").mirrorPath();
      BStat_C = PathPlannerPath.fromChoreoTrajectory("TStat-L").mirrorPath();
      C_BStat = PathPlannerPath.fromChoreoTrajectory("L-TStat").mirrorPath();
      BStat_B = PathPlannerPath.fromChoreoTrajectory("TStat-A").mirrorPath();

      TOP_PUSH = PathPlannerPath.fromChoreoTrajectory("TopPush");

    } catch (Exception e) {
      System.out.println("Failed to load paths. DO NOT RUN AUTO");
    }
  }

  public Command moveStraight() {
    return DriveCommands.joystickDrive(drive, () -> -0.2, () -> 0, () -> 0).withTimeout(2.0);
  }

  // General schema for all routines:

  // If scoring:
  // Follow path, align to branch
  // Move elevator/wrist and shoot

  // If picking up from station:
  // Move elevator/wrist down to station position
  // Follow path
  // Wait 0.5 seconds

  private Command autoAlignAndScore(AutoAlignDirection direction) {
    return Commands.sequence(
        DriveCommands.autoAlignToNearestBranchAuto(drive, direction).withTimeout(Seconds.of(0.5)),
        elevatorAndWristCommands.goToL4(),
        shooter
            .shootCoralInAutoCmd(wrist.isAtScoringState, elevator::getCurrentElevatorState)
            .withTimeout(Seconds.of(0.75)));
  }

  public Command fourPieceFromTop() {
    return Commands.sequence(
        drive.setPoseCmd(T2_J.getStartingDifferentialPose()),
        AutoBuilder.followPath(T2_J),
        autoAlignAndScore(RIGHT),
        elevatorAndWristCommands.goToStation(),
        AutoBuilder.followPath(J_TStat),
        Commands.waitTime(Seconds.of(0.5)),
        AutoBuilder.followPath(TStat_K),
        autoAlignAndScore(LEFT),
        elevatorAndWristCommands.goToStation(),
        AutoBuilder.followPath(K_TStat),
        Commands.waitTime(Seconds.of(0.5)),
        AutoBuilder.followPath(TStat_L),
        autoAlignAndScore(RIGHT),
        elevatorAndWristCommands.goToStation(),
        AutoBuilder.followPath(L_TStat),
        Commands.waitTime(Seconds.of(0.5)),
        AutoBuilder.followPath(TStat_A),
        autoAlignAndScore(LEFT),
        elevatorAndWristCommands.goToStation());
  }

  public Command fourPieceFromBottom() {
    return Commands.sequence(
        drive.setPoseCmd(B2_E.getStartingDifferentialPose()),
        AutoBuilder.followPath(B2_E),
        autoAlignAndScore(LEFT),
        elevatorAndWristCommands.goToStation(),
        AutoBuilder.followPath(E_BStat),
        Commands.waitTime(Seconds.of(0.5)),
        AutoBuilder.followPath(BStat_D),
        autoAlignAndScore(RIGHT),
        elevatorAndWristCommands.goToStation(),
        AutoBuilder.followPath(D_BStat),
        Commands.waitTime(Seconds.of(0.5)),
        AutoBuilder.followPath(BStat_C),
        autoAlignAndScore(LEFT),
        elevatorAndWristCommands.goToStation(),
        AutoBuilder.followPath(C_BStat),
        Commands.waitTime(Seconds.of(0.5)),
        AutoBuilder.followPath(BStat_B),
        autoAlignAndScore(RIGHT),
        elevatorAndWristCommands.goToStation());
  }

  public Command threePieceFromTop() {
    return Commands.sequence(
        drive.setPoseCmd(T2_J.getStartingDifferentialPose()),
        AutoBuilder.followPath(T2_J),
        autoAlignAndScore(RIGHT),
        elevatorAndWristCommands.goToStation(),
        AutoBuilder.followPath(J_TStat),
        Commands.waitTime(Seconds.of(0.5)),
        AutoBuilder.followPath(TStat_K),
        autoAlignAndScore(LEFT),
        elevatorAndWristCommands.goToStation(),
        AutoBuilder.followPath(K_TStat),
        Commands.waitTime(Seconds.of(0.5)),
        AutoBuilder.followPath(TStat_L),
        autoAlignAndScore(RIGHT),
        elevatorAndWristCommands.goToStation());
  }

  public Command threePieceFromBottom() {
    return Commands.sequence(
        drive.setPoseCmd(B2_E.getStartingDifferentialPose()),
        AutoBuilder.followPath(B2_E),
        autoAlignAndScore(LEFT),
        elevatorAndWristCommands.goToStation(),
        AutoBuilder.followPath(E_BStat),
        Commands.waitTime(Seconds.of(0.5)),
        AutoBuilder.followPath(BStat_D),
        autoAlignAndScore(RIGHT),
        elevatorAndWristCommands.goToStation(),
        AutoBuilder.followPath(D_BStat),
        Commands.waitTime(Seconds.of(0.5)),
        AutoBuilder.followPath(BStat_C),
        autoAlignAndScore(LEFT),
        elevatorAndWristCommands.goToStation());
  }

  public Command twoPieceFromTop() {
    return Commands.sequence(
        drive.setPoseCmd(T2_J.getStartingDifferentialPose()),
        AutoBuilder.followPath(T2_J),
        autoAlignAndScore(RIGHT),
        elevatorAndWristCommands.goToStation(),
        AutoBuilder.followPath(J_TStat),
        Commands.waitTime(Seconds.of(0.5)),
        AutoBuilder.followPath(TStat_K),
        autoAlignAndScore(LEFT),
        elevatorAndWristCommands.goToStation());
  }

  public Command twoPieceFromBottom() {
    return Commands.sequence(
        drive.setPoseCmd(B2_E.getStartingDifferentialPose()),
        AutoBuilder.followPath(B2_E),
        autoAlignAndScore(LEFT),
        elevatorAndWristCommands.goToStation(),
        AutoBuilder.followPath(E_BStat),
        Commands.waitTime(Seconds.of(0.5)),
        AutoBuilder.followPath(BStat_D),
        autoAlignAndScore(RIGHT),
        elevatorAndWristCommands.goToStation());
  }

  public Command onePieceFromCenter() {
    return Commands.sequence(
        drive.setPoseCmd(C4_H.getStartingDifferentialPose()),
        AutoBuilder.followPath(C4_H),
        autoAlignAndScore(RIGHT),
        elevatorAndWristCommands.goToStation());
  }

  public Command onePieceFromTop() {
    return Commands.sequence(
        drive.setPoseCmd(T2_J.getStartingDifferentialPose()),
        AutoBuilder.followPath(T2_J),
        autoAlignAndScore(RIGHT),
        elevatorAndWristCommands.goToStation());
  }

  public Command onePieceFromBottom() {
    return Commands.sequence(
        drive.setPoseCmd(B2_E.getStartingDifferentialPose()),
        AutoBuilder.followPath(B2_E),
        autoAlignAndScore(LEFT),
        elevatorAndWristCommands.goToStation());
  }

  public Command doNothing() {
    return Commands.none();
  }

  public Command pushAndOnePieceFromTop() {
    return Commands.sequence(
        drive.setPoseCmd(TOP_PUSH.getStartingDifferentialPose()),
        AutoBuilder.followPath(TOP_PUSH),
        autoAlignAndScore(RIGHT),
        elevatorAndWristCommands.goToStation());
  }

  public Command pushAndOnePieceFromBottom() {
    return Commands.sequence(
        drive.setPoseCmd(TOP_PUSH.getStartingDifferentialPose()),
        AutoBuilder.followPath(TOP_PUSH),
        autoAlignAndScore(LEFT),
        elevatorAndWristCommands.goToStation());
  }
}
