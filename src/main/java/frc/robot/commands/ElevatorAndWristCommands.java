package frc.robot.commands;

import static frc.robot.Constants.TargetState.*;
import static frc.robot.RobotContainer.targetPosition;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ScoringPieceType;
import frc.robot.Constants.TargetState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants.WristState;
import java.util.Set;

/** A class that contains commands for moving the elevator and wrist to specific states. */
public class ElevatorAndWristCommands {
  private final Elevator elevator;
  private final Wrist wrist;

  public ElevatorAndWristCommands(Elevator elevator, Wrist wrist) {
    this.elevator = elevator;
    this.wrist = wrist;

    SmartDashboard.putData("Stow E+W", stowAllReset());
  }

  private boolean isAlgaeState(TargetState targetState) {
    return targetState == ALGAE_STOW
        || targetState == PROCESSOR
        || targetState == L2_ALGAE
        || targetState == L3_ALGAE
        || targetState == NET;
  }

  private boolean isCoralState(TargetState targetState) {
    return targetState == STATION
        || targetState == L1
        || targetState == L2_CORAL
        || targetState == L3_CORAL
        || targetState == L4;
  }

  /**
   * Moves the elevator after stowing wrist, then moves the wrist to the target state. This is used
   * when moving:
   *
   * <p>- between coral states,
   *
   * <p>- between coral and algae states on different levels
   *
   * <p>- and when moving to the station.
   */
  private Command goToStateWithPreStow(TargetState targetState) {
    return wrist
        .runTargetState(WristState.STATION)
        .until(wrist.isAtStation)
        .andThen(
            elevator
                .runTargetState(targetState.elevatorState)
                .asProxy()
                .until(elevator.isAtTargetState))
        .andThen(wrist.runTargetState(targetState.wristState));
  }

  private Command goToStateWithLateUnstow(TargetState targetState, Trigger aboveState) {
    return elevator
        .runTargetState(targetState.elevatorState)
        .alongWith(
            Commands.waitUntil(aboveState).andThen(wrist.runTargetState(targetState.wristState)));
  }

  /**
   * Moves the elevator and the wrist simultaneously without stowing first. This is used when
   * moving:
   *
   * <ul>
   *   <li>between algae states
   *   <li>between algae and coral states on the same level (excluding L4/net)
   */
  private Command goToStateDirect(TargetState targetState) {
    return elevator
        .runTargetState(targetState.elevatorState)
        .alongWith(wrist.runTargetState(targetState.wristState));
  }

  /* Move to station with coral state movement, setting the target position first. */
  public Command goToStation() {
    return Commands.defer(
            () -> {
              // If we're already at station, do nothing
              if (targetPosition == STATION) {
                return Commands.none();
              }
              // if previous state is processor/L1, move up a bit to safely stow wrist, then move to
              // station
              if (targetPosition == PROCESSOR || targetPosition == L1) {
                return goToL1Intermediate()
                    .andThen(
                        Commands.runOnce(() -> targetPosition = STATION)
                            .andThen(goToStateWithPreStow(STATION)));
              }
              // ensure wrist and elevator move here at coral speed
              wrist.setToCoralSpeed();
              elevator.setToCoralSpeed();

              // If previous state is algae stow/L1_intermediate, prestow wrist before moving
              // elevator down
              if (targetPosition == ALGAE_STOW || targetPosition == L1_INTERMEDIATE) {
                targetPosition = STATION;
                return goToStateWithPreStow(STATION);
              } else {
                // otherwise simultaneously move wrist and elevator
                targetPosition = STATION;
                return goToStateDirect(STATION);
              }
            },
            Set.of(elevator, wrist))
        .onlyWhile(elevator.isAtTargetState.negate().or(wrist.isAtStation.negate()));
  }

  /* Move to L1 with appropriate state movement. */
  public Command goToL1() {
    return Commands.defer(
            () -> {
              // If we're already at L1, do nothing
              if (targetPosition == L1) {
                return Commands.none();
              }

              // if at station, move up a little bit first
              if (targetPosition == STATION) {
                return goToL1Intermediate().andThen(goToL1());
              }
              // otherwise simultaneously move wrist and elevator
              else {
                targetPosition = L1;
                return goToStateDirect(L1);
              }
            },
            Set.of(elevator, wrist))
        .onlyWhile(elevator.isAtTargetState.negate().or(wrist.isAtScoringState.negate()));
  }

  private Command goToL1Intermediate() {
    // move elevator to L1_intermediate
    return (Commands.run(() -> targetPosition = L1_INTERMEDIATE)
            .alongWith(elevator.runTargetState(ElevatorState.L1_INTERMEDIATE)))
        .onlyWhile(elevator.isAtTargetState.negate());
  }

  /* Move to algae stow with algae state movement. */
  public Command goToAlgaeStow() {
    return goToAlgae(ALGAE_STOW, true);
  }

  /* Move to processor with appropriate state movement. */
  public Command goToProcessor() {
    return Commands.defer(
            () -> {
              // if at station, move a little bit up first
              if (targetPosition == STATION) {
                return goToL1Intermediate()
                    .andThen(
                        // move wrist to processor, then elevator to processor
                        wrist
                            .runTargetState(WristState.PROCESSOR)
                            .until(wrist.isAtScoringState)
                            .andThen(goToProcessor()));
              }
              return goToAlgae(PROCESSOR, false);
            },
            Set.of(elevator, wrist))
        .onlyWhile(elevator.isAtTargetState.negate().or(wrist.isAtScoringState.negate()));
  }

  /* Move to L2_coral with appropriate state movement. */
  public Command goToL2Coral() {
    return goToCoral(L2_CORAL, false);
  }

  /* Move to L2_algae with appropriate state movement. */
  public Command goToL2Algae() {
    return goToAlgae(L2_ALGAE, false);
  }

  /* Move to L3_coral with appropriate state movement. */
  public Command goToL3Coral() {
    return goToCoral(L3_CORAL, false);
  }

  /* Move to L3_algae with appropriate state movement. */
  public Command goToL3Algae() {
    return goToAlgae(L3_ALGAE, false);
  }

  /* Move to net with appropriate state movement. */
  public Command goToNet() {
    return goToAlgae(NET, false);
  }

  /* Move to L4 with coral state movement. */
  public Command goToL4() {
    return goToCoral(L4, true);
  }

  /* Helper function for setting elevator and wrist to move at algae speeds */
  private void setToAlgaeSpeeds() {
    wrist.setToAlgaeSpeed();
    // elevator.setToAlgaeSpeed();
  }

  private Command goToAlgae(TargetState targetState, boolean directExplicit) {
    return Commands.defer(
            () -> {
              // If we're already at the target state, do nothing
              if (targetPosition == targetState) {
                return Commands.none();
              }
              // if the previous state was algae, ensure wrist and elevator move here at algae speed
              if (isAlgaeState(targetPosition)) {
                setToAlgaeSpeeds();
              } else {
                wrist.setToCoralSpeed();
              }

              // if the previous state was coral, move the wrist and elevator with late unstow
              // But if the previous state was algae, we can simultaneously move the wrist and
              // elevator
              if (directExplicit) {
                targetPosition = targetState;
                return goToStateDirect(targetState);
              } else if (isCoralState(targetPosition)) {
                targetPosition = targetState;
                return goToStateWithLateUnstow(targetState, elevator.isAboveL1Intermediate);
              } else {
                targetPosition = targetState;
                return goToStateDirect(targetState);
              }
            },
            Set.of(elevator, wrist))
        .onlyWhile(elevator.isAtTargetState.negate().or(wrist.isAtScoringState.negate()));
  }

  private Command goToCoral(TargetState targetState, boolean prestowL4) {
    return Commands.defer(
            () -> {
              // If we're already at the target state, do nothing
              if (targetPosition == targetState) {
                return Commands.none();
              }
              // if the previous state was algae, ensure wrist and elevator move here at coral speed
              if (isAlgaeState(targetPosition)) {
                wrist.setToCoralSpeed();
                elevator.setToCoralSpeed();
              }

              if (prestowL4) {
                targetPosition = targetState;
                return goToStateWithLateUnstow(targetState, elevator.isAboveL3Intermediate);
              } else {
                targetPosition = targetState;
                return goToStateWithLateUnstow(targetState, elevator.isAboveL1Intermediate);
              }
            },
            Set.of(elevator, wrist))
        .onlyWhile(elevator.isAtTargetState.negate().or(wrist.isAtScoringState.negate()));
  }

  /**
   * Sets the current scoring piece type to algae. If the target position is L2 or L3, move to the
   * other state.
   *
   * @return
   */
  public Command setScoringPieceToAlgae() {
    return Commands.defer(
        () -> {
          RobotContainer.currentScoringPieceType = ScoringPieceType.ALGAE;
          if (targetPosition == L2_CORAL) {
            return goToL2Algae();
          } else if (targetPosition == L3_CORAL) {
            return goToL3Algae();
          }
          return Commands.none();
        },
        Set.of(elevator, wrist));
  }

  public Command setScoringPieceToCoral() {
    return Commands.runOnce(() -> RobotContainer.currentScoringPieceType = ScoringPieceType.CORAL);
  }

  public Command stowAllReset() {
    // return Commands.sequence(
    //     Commands.runOnce(() -> RobotContainer.currentScoringPieceType = ScoringPieceType.CORAL),
    //     wrist.runTargetState(WristState.STATION),
    //     Commands.waitSeconds(1.0),
    //     elevator.setTargetState(ElevatorState.STATION));
    return Commands.runOnce(() -> RobotContainer.currentScoringPieceType = ScoringPieceType.CORAL)
        .andThen(
            wrist
                .runTargetState(WristState.STATION)
                .until(wrist.isAtStation)
                .andThen(
                    elevator
                        .runTargetState(ElevatorState.STATION)
                        .until(elevator.isAtTargetState)));
  }
}
