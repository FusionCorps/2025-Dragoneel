package frc.robot.commands;

import static frc.robot.Constants.ScoringModeState.*;
import static frc.robot.Robot.currentScoringMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  }

  /*
   * Stow the wrist, then move the elevator, then move the wrist.
   * This is used when moving between coral states.
   */
  private Command goToStateFromCoral(WristState wristState, ElevatorState elevatorState) {
    return Commands.sequence(
        wrist.goToState(WristState.STATION),
        Commands.waitUntil(wrist.isAtStation),
        elevator.goToState(elevatorState),
        Commands.waitUntil(elevator.isAtTargetState),
        wrist.goToState(wristState));
  }

  /*
   * Move the elevator, then move the wrist. Do not stow the wrist ever.
   * This is used when moving between algae states.
   */
  private Command goToStateFromAlgae(WristState wristState, ElevatorState elevatorState) {
    return Commands.sequence(
        elevator.goToState(elevatorState),
        Commands.waitUntil(elevator.isAtTargetState),
        wrist.goToState(wristState));
  }

  /* Move to station with coral state movement. */
  public Command goToStation() {
    return Commands.either(
        Commands.none(),
        goToStateFromCoral(WristState.STATION, ElevatorState.STATION)
            .alongWith(Commands.runOnce(() -> currentScoringMode = STATION)),
        () -> currentScoringMode == STATION);
  }

  /* Move to L1 with appropriate state movement based on current scoring mode. */
  public Command goToL1() {
    return Commands.defer(
        () -> {
          if (currentScoringMode == L1) return Commands.none();
          Command command =
              (currentScoringMode == PROCESSOR)
                  ? goToStateFromAlgae(WristState.L1, ElevatorState.L1)
                  : goToStateFromCoral(WristState.L1, ElevatorState.L1);
          return command.alongWith(Commands.runOnce(() -> currentScoringMode = L1));
        },
        Set.of(wrist, elevator));
  }

  /* Move to processor with appropriate state movement based on current scoring mode. */
  public Command goToProcessor() {
    return Commands.defer(
        () -> {
          if (currentScoringMode == PROCESSOR) return Commands.none();
          Command command =
              (currentScoringMode == L1
                      || currentScoringMode == L2_ALGAE
                      || currentScoringMode == L3_ALGAE
                      || currentScoringMode == NET)
                  ? goToStateFromAlgae(WristState.PROCESSOR, ElevatorState.PROCESSOR)
                  : goToStateFromCoral(WristState.PROCESSOR, ElevatorState.PROCESSOR);
          return command.alongWith(Commands.runOnce(() -> currentScoringMode = PROCESSOR));
        },
        Set.of(wrist, elevator));
  }

  /* Move to L2_coral with appropriate state movement based on current scoring mode. */
  public Command goToL2Coral() {
    return Commands.defer(
        () -> {
          if (currentScoringMode == L2_CORAL) return Commands.none();
          Command command =
              (currentScoringMode == L2_ALGAE)
                  ? goToStateFromAlgae(WristState.L2_CORAL, ElevatorState.L2)
                  : goToStateFromCoral(WristState.L2_CORAL, ElevatorState.L2);
          return command.alongWith(Commands.runOnce(() -> currentScoringMode = L2_CORAL));
        },
        Set.of(wrist, elevator));
  }

  /* Move to L2_algae with appropriate state movement based on current scoring mode. */
  public Command goToL2Algae() {
    return Commands.defer(
        () -> {
          if (currentScoringMode == L2_ALGAE) return Commands.none();
          Command command =
              (currentScoringMode == L2_CORAL
                      || currentScoringMode == L3_ALGAE
                      || currentScoringMode == NET
                      || currentScoringMode == PROCESSOR)
                  ? goToStateFromAlgae(WristState.L2_ALGAE, ElevatorState.L2)
                  : goToStateFromCoral(WristState.L2_ALGAE, ElevatorState.L2);
          return command.alongWith(Commands.runOnce(() -> currentScoringMode = L2_ALGAE));
        },
        Set.of(wrist, elevator));
  }

  /* Move to L3_coral with appropriate state movement based on current scoring mode. */
  public Command goToL3Coral() {
    return Commands.defer(
        () -> {
          if (currentScoringMode == L3_CORAL) return Commands.none();
          Command command =
              (currentScoringMode == L3_ALGAE)
                  ? goToStateFromAlgae(WristState.L3_CORAL, ElevatorState.L3)
                  : goToStateFromCoral(WristState.L3_CORAL, ElevatorState.L3);
          return command.alongWith(Commands.runOnce(() -> currentScoringMode = L3_CORAL));
        },
        Set.of(wrist, elevator));
  }

  /* Move to L3_algae with algae state movement if starting at L3_coral/algae states and coral state movement otherwise. */
  public Command goToL3Algae() {
    return Commands.defer(
        () -> {
          if (currentScoringMode == L3_ALGAE) return Commands.none();
          Command command =
              (currentScoringMode == L3_CORAL
                      || currentScoringMode == NET
                      || currentScoringMode == L2_ALGAE
                      || currentScoringMode == PROCESSOR)
                  ? goToStateFromAlgae(WristState.L3_ALGAE, ElevatorState.L3)
                  : goToStateFromCoral(WristState.L3_ALGAE, ElevatorState.L3);
          return command.alongWith(Commands.runOnce(() -> currentScoringMode = L3_ALGAE));
        },
        Set.of(wrist, elevator));
  }

  /* Move to net with algae state movement if starting at algae states and coral state movement otherwise. */
  public Command goToNet() {
    return Commands.defer(
        () -> {
          if (currentScoringMode == NET) return Commands.none();
          Command command =
              (currentScoringMode == L2_ALGAE
                      || currentScoringMode == L3_ALGAE
                      || currentScoringMode == PROCESSOR)
                  ? goToStateFromAlgae(WristState.NET, ElevatorState.NET)
                  : goToStateFromCoral(WristState.NET, ElevatorState.NET);
          return command.alongWith(Commands.runOnce(() -> currentScoringMode = NET));
        },
        Set.of(wrist, elevator));
  }

  /* Move to L4 with coral state movement. */
  public Command goToL4() {
    return Commands.either(
        Commands.none(),
        goToStateFromCoral(WristState.L4, ElevatorState.L4)
            .alongWith(Commands.runOnce(() -> currentScoringMode = L4)),
        () -> currentScoringMode == L4);
  }
}
