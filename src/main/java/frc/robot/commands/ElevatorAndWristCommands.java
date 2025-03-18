package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.TargetState.*;

import edu.wpi.first.wpilibj.util.Color;
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
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** A class that contains commands for moving the elevator and wrist to specific states. */
public class ElevatorAndWristCommands {
  private final Elevator elevator;
  private final Wrist wrist;

  @AutoLogOutput(key = "TargetStates/PreviousTargetState")
  private TargetState previousTargetState = STATION;

  @AutoLogOutput(key = "TargetStates/CurrentTargetState")
  private TargetState currentTargetState = STATION;

  /**
   * Tracks which scoring mode the robot is in. The robot has ONLY two scoring modes, {@link
   * ScoringPieceType#CORAL} and {@link ScoringPieceType#ALGAE}
   */
  private ScoringPieceType scoringPieceType = ScoringPieceType.CORAL;

  public Trigger isInCoralMode = new Trigger(() -> scoringPieceType == ScoringPieceType.CORAL);
  public Trigger isInAlgaeMode = new Trigger(() -> scoringPieceType == ScoringPieceType.ALGAE);

  public ElevatorAndWristCommands(Elevator elevator, Wrist wrist) {
    this.elevator = elevator;
    this.wrist = wrist;

    this.wrist.setScoringPieceTypeSupplier(() -> scoringPieceType);

    isInAlgaeMode.onTrue(RobotContainer.rumbleCommand());

    isInCoralMode
        .whileTrue(
            Commands.runOnce(
                () -> Logger.recordOutput("Current Scoring Type", Color.kWhite.toHexString())))
        .onFalse(Commands.runOnce(() -> Logger.recordOutput("Current Scoring Type", "#29ac9c")));
  }

  /** Helper method to determine if a state is an algae position */
  private boolean isAlgaePosition(TargetState state) {
    return state == PROCESSOR
        || state == L2_ALGAE
        || state == L3_ALGAE
        || state == NET
        || state == ALGAE_STOW;
  }

  /** Manages state transitions and applies appropriate speeds */
  private Command transitionToState(TargetState newTargetState) {
    return Commands.deferredProxy(
        () -> {
          // If no change in state, do nothing
          if (newTargetState == currentTargetState) {
            return Commands.none();
          }

          // Save previous state before updating current
          previousTargetState = currentTargetState;
          currentTargetState = newTargetState;

          // Special case: always use default movement when going to STATION
          if (newTargetState == STATION) {
            return goToStateDefault(newTargetState).beforeStarting(setToCoralSpeeds());
          }

          // Determine transition type and apply appropriate movement pattern
          Command moveCommand;
          boolean wasAlgaePosition = isAlgaePosition(previousTargetState);
          boolean isAlgaePosition = isAlgaePosition(newTargetState);

          // If moving between algae positions, use default movement with algae speeds
          if (wasAlgaePosition && isAlgaePosition && scoringPieceType == ScoringPieceType.ALGAE) {
            moveCommand = goToStateDefault(newTargetState).beforeStarting(setToAlgaeSpeeds());
          }
          // If moving from corresponding coral to algae, use default movement with coral speeds
          else if (!wasAlgaePosition && isAlgaePosition) {
            moveCommand = goToStateDefault(newTargetState).beforeStarting(setToCoralSpeeds());
          }
          // Stow first when moving between station and L1/processor/algae stow
          else if (previousTargetState == STATION
                  && (newTargetState == L1
                      || newTargetState == PROCESSOR
                      || newTargetState == ALGAE_STOW)
              || newTargetState == STATION
                  && (previousTargetState == L1
                      || previousTargetState == PROCESSOR
                      || previousTargetState == ALGAE_STOW)) {
            moveCommand = goToStateWithStow(newTargetState).beforeStarting(setToCoralSpeeds());
          }
          // For all other cases, use default movement with coral speeds
          else {
            moveCommand = goToStateDefault(newTargetState).beforeStarting(setToCoralSpeeds());
          }

          return moveCommand;
        });
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
  private Command goToStateWithStow(TargetState targetState) {
    return Commands.sequence(
        wrist.setTargetState(WristState.STATION),
        Commands.waitUntil(wrist.hasReachedStation),
        elevator.setTargetState(targetState.elevatorState),
        Commands.waitUntil(elevator.hasReachedTargetState),
        wrist.setTargetState(targetState.wristState));
  }

  /**
   * Moves the elevator, then the wrist without stowing first. This is used when moving:
   *
   * <p>- between algae states
   *
   * <p>- between algae and coral states on the same level (excluding L4/net)
   */
  // private Command goToStateDefault(TargetState targetState) {
  //   // Special handling for ALGAE_STOW to ensure wrist movement completes
  //   if (targetState == ALGAE_STOW) {
  //     return Commands.sequence(
  //         elevator.setTargetState(targetState.elevatorState),
  //         Commands.waitUntil(elevator.hasReachedTargetState),
  //         wrist.setTargetState(targetState.wristState),
  //         // Add explicit wait for wrist to complete moving
  //         Commands.waitUntil(() -> wrist.getCurrentWristState() == targetState.wristState));
  //   }

  //   // Normal behavior for other targets
  //   return Commands.sequence(
  //       Commands.print("a1"),
  //       elevator.setTargetState(targetState.elevatorState),
  //       Commands.print("a2"),
  //       Commands.waitUntil(elevator.hasReachedTargetState),
  //       Commands.print("a3"),
  //       wrist.setTargetState(targetState.wristState));
  // }

  private Command goToStateDefault(TargetState targetState) {
    return Commands.sequence(
        elevator.setTargetState(targetState.elevatorState),
        Commands.waitTime(Seconds.of(0.05)),
        wrist.setTargetState(targetState.wristState));
  }

  /* Move to station */
  public Command goToStation() {
    return transitionToState(STATION);
  }

  public Command goToProcessor() {
    return transitionToState(PROCESSOR);
  }

  public Command goToL1() {
    return transitionToState(L1);
  }

  public Command goToL2Coral() {
    return transitionToState(L2_CORAL);
  }

  public Command goToL2Algae() {
    return transitionToState(L2_ALGAE);
  }

  public Command goToL3Coral() {
    return transitionToState(L3_CORAL);
  }

  public Command goToL3Algae() {
    return transitionToState(L3_ALGAE);
  }

  public Command goToL4() {
    return transitionToState(L4);
  }

  public Command goToNet() {
    return transitionToState(NET);
  }

  public Command goToAlgaeStow() {
    return transitionToState(ALGAE_STOW);
  }

  public Command setNeutral() {
    return wrist
        .setTargetState(WristState.NEUTRAL)
        .alongWith(elevator.setTargetState(ElevatorState.NEUTRAL));
  }

  /**
   * Sets the scoring piece type to algae and transitions to appropriate algae position if needed
   */
  public Command setScoringModeToAlgae() {
    return Commands.defer(
        () -> {
          // Update scoring mode
          scoringPieceType = ScoringPieceType.ALGAE;

          // If already in algae position, do nothing
          if (isAlgaePosition(currentTargetState)) {
            return Commands.none();
          }

          // Otherwise, transition to appropriate algae position
          if (currentTargetState == L2_CORAL) {
            return transitionToState(L2_ALGAE);
          } else if (currentTargetState == L3_CORAL) {
            return transitionToState(L3_ALGAE);
          } else if (currentTargetState == L1) {
            return transitionToState(PROCESSOR);
          }

          return Commands.none();
        },
        Set.of(elevator, wrist));
  }

  /** Sets the scoring piece type to coral */
  public Command setScoringModeToCoral() {
    return Commands.runOnce(
        () -> {
          scoringPieceType = ScoringPieceType.CORAL;
        });
  }

  /**
   * @return the current target state
   */
  public TargetState getCurrentTargetState() {
    return currentTargetState;
  }

  /**
   * @return the previous target state
   */
  public TargetState getPreviousTargetState() {
    return previousTargetState;
  }

  /**
   * @return the current scoring piece type
   */
  public ScoringPieceType getScoringPieceType() {
    return scoringPieceType;
  }

  /** Sets elevator and wrist to coral speeds */
  public Command setToCoralSpeeds() {
    return elevator.setToCoralSpeed().alongWith(wrist.setToCoralSpeed());
  }

  /** Sets elevator and wrist to algae speeds */
  public Command setToAlgaeSpeeds() {
    return elevator.setToAlgaeSpeed().alongWith(wrist.setToAlgaeSpeed());
  }
}
