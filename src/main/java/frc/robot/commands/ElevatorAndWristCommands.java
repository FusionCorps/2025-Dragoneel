package frc.robot.commands;

import static frc.robot.Constants.TargetState.*;
import static frc.robot.RobotContainer.targetPosition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ScoringPieceType;
import frc.robot.Constants.TargetState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.Elevator;
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
  private Command goToStateDefault(TargetState targetState) {
    return Commands.sequence(
        wrist.setTargetState(WristState.STATION),
        Commands.waitUntil(wrist.isAtStation),
        elevator.setTargetState(targetState.elevatorState),
        Commands.waitUntil(elevator.isAtTargetState),
        wrist.setTargetState(targetState.wristState));
  }

  /**
   * Moves the elevator, then the wrist without stowing first. This is used when moving:
   *
   * <p>- between algae states
   *
   * <p>- between algae and coral states on the same level (excluding L4/net)
   */
  private Command goToStateBetweenAlgae(TargetState targetState) {
    return Commands.sequence(
        elevator.setTargetState(targetState.elevatorState),
        Commands.waitUntil(elevator.isAtTargetState),
        wrist.setTargetState(targetState.wristState));
  }

  /* Move to station with coral state movement, setting the target position first. */
  public Command goToStation() {
    return Commands.defer(
        () -> {
          // If we're already at station, do nothing
          if (targetPosition == STATION) {
            return Commands.none();
          }
          // Otherwise set the target position first, then move
          targetPosition = STATION;
          return goToStateDefault(targetPosition);
        },
        Set.of());
  }

  /* Move to L1 with appropriate state movement. */
  public Command goToL1() {
    return Commands.defer(
        () -> {
          // If we're already at L1, do nothing
          if (targetPosition == L1) {
            return Commands.none();
          }
          // Otherwise remember the old target, then set the new target
          targetPosition = L1;
          // Decide how to move based on old target
          return goToStateDefault(targetPosition);
        },
        Set.of());
  }

  /* Move to processor with appropriate state movement. */
  public Command goToProcessor() {
    return Commands.defer(
        () -> {
          // If we're already at PROCESSOR, do nothing
          if (targetPosition == PROCESSOR) {
            return Commands.none();
          }
          targetPosition = PROCESSOR;
          return goToStateBetweenAlgae(targetPosition);
        },
        Set.of());
  }

  /* Move to L2_coral with appropriate state movement. */
  public Command goToL2Coral() {
    return Commands.defer(
        () -> {
          // If we're already at L2_CORAL, do nothing
          if (targetPosition == L2_CORAL) {
            return Commands.none();
          }
          targetPosition = L2_CORAL;
          return goToStateDefault(targetPosition);
        },
        Set.of());
  }

  /* Move to L2_algae with appropriate state movement. */
  public Command goToL2Algae() {
    return Commands.defer(
        () -> {
          // If we're already at L2_ALGAE, do nothing
          if (targetPosition == L2_ALGAE) {
            return Commands.none();
          }
          targetPosition = L2_ALGAE;
          return goToStateBetweenAlgae(targetPosition);
        },
        Set.of());
  }

  /* Move to L3_coral with appropriate state movement. */
  public Command goToL3Coral() {
    return Commands.defer(
        () -> {
          // If we're already at L3_CORAL, do nothing
          if (targetPosition == L3_CORAL) {
            return Commands.none();
          }

          // Otherwise set movement method to L3 coral based on old target
          targetPosition = L3_CORAL;
          return goToStateDefault(targetPosition);
        },
        Set.of());
  }

  /* Move to L3_algae with appropriate state movement. */
  public Command goToL3Algae() {
    return Commands.defer(
        () -> {
          // If we're already at L3_ALGAE, do nothing
          if (targetPosition == L3_ALGAE) {
            return Commands.none();
          }
          // Otherwise set movement type to L3 algae based on old target
          targetPosition = L3_ALGAE;
          return goToStateBetweenAlgae(targetPosition);
        },
        Set.of());
  }

  /* Move to net with appropriate state movement. */
  public Command goToNet() {
    return Commands.defer(
        () -> {
          // If we're already at NET, do nothing
          if (targetPosition == NET) {
            return Commands.none();
          }
          // Otherwise set movement type to NET based on old targetw
          targetPosition = NET;
          return goToStateBetweenAlgae(targetPosition);
        },
        Set.of());
  }

  /* Move to L4 with coral state movement. */
  public Command goToL4() {
    return Commands.defer(
        () -> {
          // If we're already at L4, do nothing
          if (targetPosition == L4) {
            return Commands.none();
          }
          // Otherwise move to L4 with stowing movement
          targetPosition = L4;
          return goToStateDefault(targetPosition);
        },
        Set.of());
  }

  /* Move to algae stow with algae state movement. */
  public Command goToAlgaeStow() {
    return Commands.defer(
        () -> {
          if (targetPosition == ALGAE_STOW) {
            return Commands.none();
          }
          targetPosition = ALGAE_STOW;
          return goToStateBetweenAlgae(targetPosition);
        },
        Set.of());
  }

  /**
   * Sets the scoring piece type to algae. If the target position is L1 or L2 or L3, move to the
   * other state.
   *
   * @return
   */
  public Command setScoringPieceToAlgae() {
    return Commands.defer(
        () -> {
          if (RobotContainer.currentScoringPieceType == ScoringPieceType.CORAL) {
            RobotContainer.currentScoringPieceType = ScoringPieceType.ALGAE;
            if (targetPosition == L1) {
              return goToProcessor();
            } else if (targetPosition == L2_CORAL) {
              return goToL2Algae();
            } else if (targetPosition == L3_CORAL) {
              return goToL3Algae();
            }
          }
          return Commands.none();
        },
        Set.of());
  }
}
