package frc.robot.commands;

import static frc.robot.Constants.TargetState.*;
import static frc.robot.RobotContainer.targetPosition;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        .andThen(elevator.runTargetState(targetState.elevatorState).until(elevator.isAtTargetState))
        .andThen(wrist.runTargetState(targetState.wristState));
  }

  private Command goToStateWithStowAlt(TargetState targetState) {
    return elevator
        .runTargetState(targetState.elevatorState)
        .alongWith(
            Commands.waitUntil(elevator.isAboveL1Intermediate)
                .andThen(wrist.runTargetState(targetState.wristState)));
  }

  /**
   * Moves the elevator and the wrist simultaneously without stowing first. This is used when
   * moving:
   *
   * <p>- between algae states
   *
   * <p>- between algae and coral states on the same level (excluding L4/net)
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
              // if at processor or L1, move up a little to safely stow wrist then move to station
              if (targetPosition == PROCESSOR || targetPosition == L1) {
                return goToL1Intermediate().until(elevator.isAtTargetState).andThen(goToStation());
              }
              wrist.setToCoralSpeed();
              elevator.setToCoralSpeed();
              // If at algae stow or L1_intermediate prestow the wrist before moving elevator back
              // down
              if (targetPosition == ALGAE_STOW || targetPosition == L1_INTERMEDIATE) {
                targetPosition = STATION;
                return goToStateWithPreStow(STATION);
              }
              // otherwise simultaneously move wrist and elevator
              targetPosition = STATION;
              return goToStateDirect(STATION);
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
                return goToL1Intermediate().until(elevator.isAtTargetState).andThen(goToL1());
              }
              // otherwise simultaneously move wrist and elevator
              targetPosition = L1;
              return goToStateDirect(L1);
            },
            Set.of(elevator, wrist))
        .onlyWhile(elevator.isAtTargetState.negate().or(wrist.isAtScoringState.negate()));
  }

  private Command goToL1Intermediate() {
    return Commands.runOnce(() -> targetPosition = L1_INTERMEDIATE)
        .andThen(elevator.runTargetState(L1_INTERMEDIATE.elevatorState));
  }

  /* Move to processor with appropriate state movement. */
  public Command goToProcessor() {
    return Commands.defer(
            () -> {
              // If we're already at PROCESSOR, do nothing
              if (targetPosition == PROCESSOR) {
                return Commands.none();
              }
              // if at station, move a little bit up first
              if (targetPosition == STATION) {
                return goToL1Intermediate()
                    .until(elevator.isAtTargetState)
                    .andThen(goToProcessor());
              }
              // if moving from previous algae state, slow down wrist and elevator
              if (isAlgaeState(targetPosition)) {
                wrist.setToAlgaeSpeed();
                elevator.setToAlgaeSpeed();
              }
              targetPosition = PROCESSOR;
              // Simultaneously move wrist and elevator
              return goToStateDirect(PROCESSOR);
            },
            Set.of(elevator, wrist))
        .onlyWhile(elevator.isAtTargetState.negate().or(wrist.isAtScoringState.negate()));
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
              return goToStateWithStowAlt(L2_CORAL);
            },
            Set.of(elevator, wrist))
        .onlyWhile(elevator.isAtTargetState.negate().or(wrist.isAtScoringState.negate()));
  }

  /* Move to L2_algae with appropriate state movement. */
  public Command goToL2Algae() {
    return Commands.defer(
            () -> {
              // If we're already at L2_ALGAE, do nothing
              if (targetPosition == L2_ALGAE) {
                return Commands.none();
              }
              if (isAlgaeState(targetPosition)) {
                wrist.setToAlgaeSpeed();
                elevator.setToAlgaeSpeed();
              }
              targetPosition = L2_ALGAE;
              return goToStateDirect(L2_ALGAE);
            },
            Set.of(elevator, wrist))
        .onlyWhile(elevator.isAtTargetState.negate().or(wrist.isAtScoringState.negate()));
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
              return goToStateWithStowAlt(L3_CORAL);
            },
            Set.of(elevator, wrist))
        .onlyWhile(elevator.isAtTargetState.negate().or(wrist.isAtScoringState.negate()));
  }

  /* Move to L3_algae with appropriate state movement. */
  public Command goToL3Algae() {
    return Commands.defer(
            () -> {
              // If we're already at L3_ALGAE, do nothing
              if (targetPosition == L3_ALGAE) {
                return Commands.none();
              }
              if (isAlgaeState(targetPosition)) {
                wrist.setToAlgaeSpeed();
                elevator.setToAlgaeSpeed();
              }
              // Otherwise set movement type to L3 algae based on old target
              targetPosition = L3_ALGAE;
              return goToStateDirect(L3_ALGAE);
            },
            Set.of(elevator, wrist))
        .onlyWhile(elevator.isAtTargetState.negate().or(wrist.isAtScoringState.negate()));
  }

  /* Move to net with appropriate state movement. */
  public Command goToNet() {
    return Commands.defer(
            () -> {
              // If we're already at NET, do nothing
              if (targetPosition == NET) {
                return Commands.none();
              }
              if (isAlgaeState(targetPosition)) {
                wrist.setToAlgaeSpeed();
                elevator.setToAlgaeSpeed();
              }
              // Otherwise set movement type to NET based on old targetw
              targetPosition = NET;
              return goToStateDirect(NET);
            },
            Set.of(elevator, wrist))
        .onlyWhile(elevator.isAtTargetState.negate().or(wrist.isAtScoringState.negate()));
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
              return goToStateWithStowAlt(L4)
                  .alongWith(Commands.runOnce(() -> targetPosition = L4));
            },
            Set.of(elevator, wrist))
        .onlyWhile(elevator.isAtTargetState.negate().or(wrist.isAtScoringState.negate()));
  }

  /* Move to algae stow with algae state movement. */
  public Command goToAlgaeStow() {
    return Commands.defer(
            () -> {
              if (targetPosition == ALGAE_STOW) {
                return Commands.none();
              }
              if (isAlgaeState(targetPosition)) {
                wrist.setToAlgaeSpeed();
                elevator.setToAlgaeSpeed();
              }
              targetPosition = ALGAE_STOW;
              return goToStateDirect(ALGAE_STOW);
            },
            Set.of(elevator, wrist))
        .onlyWhile(elevator.isAtTargetState.negate().or(wrist.isAtScoringState.negate()));
  }

  public Command setNeutral() {
    return elevator
        .runTargetState(ElevatorState.NEUTRAL)
        .alongWith(wrist.runTargetState(WristState.NEUTRAL));
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
          RobotContainer.currentScoringPieceType = ScoringPieceType.ALGAE;
          if (targetPosition == L1) {
            return goToProcessor();
          } else if (targetPosition == L2_CORAL) {
            return goToL2Algae();
          } else if (targetPosition == L3_CORAL) {
            return goToL3Algae();
          }
          return Commands.none();
        },
        Set.of(elevator, wrist));
  }

  public Command setScoringPieceToCoral() {
    return Commands.defer(
        () -> {
          if (RobotContainer.currentScoringPieceType == ScoringPieceType.ALGAE) {
            RobotContainer.currentScoringPieceType = ScoringPieceType.CORAL;
            // if (targetPosition == L1) {
            //   return goToProcessor();
            // } else if (targetPosition == L2_CORAL) {
            //   return goToL2Algae();
            // } else if (targetPosition == L3_CORAL) {
            //   return goToL3Algae();
            // }
          }
          return goToStation();
        },
        Set.of());
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
