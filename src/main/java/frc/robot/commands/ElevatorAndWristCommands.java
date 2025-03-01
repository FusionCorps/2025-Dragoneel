package frc.robot.commands;

import static frc.robot.Constants.ScoringModeState.*;
import static frc.robot.Robot.currentScoringMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants.WristState;
import java.util.Arrays;
import java.util.HashSet;

public class ElevatorAndWristCommands {
  private final Elevator elevator;
  private final Wrist wrist;

  public ElevatorAndWristCommands(Elevator elevator, Wrist wrist) {
    this.elevator = elevator;
    this.wrist = wrist;
  }

  /*
   * Stow the wrist, then move the elevator, then move the wrist.
   */
  private Command goToStateFromCoral(WristState wristState, ElevatorState elevatorState) {
    return Commands.sequence(
        elevator.toggleElevatorSpeed(),
        wrist.goToState(WristState.STATION),
        Commands.waitUntil(wrist.isAtStation),
        elevator.goToState(elevatorState),
        Commands.waitUntil(elevator.isAtTargetState),
        wrist.goToState(wristState));
  }

  private Command goToStateFromAlgae(WristState wristState, ElevatorState elevatorState) {
    return Commands.sequence(
        elevator.toggleElevatorSpeed(),
        elevator.goToState(elevatorState),
        Commands.waitUntil(elevator.isAtTargetState),
        wrist.goToState(wristState));
  }

  public Command goToStation() {
    return Commands.either(
        Commands.none(),
        goToStateFromCoral(WristState.STATION, ElevatorState.STATION)
            .alongWith(Commands.runOnce(() -> currentScoringMode = STATION)),
        () -> currentScoringMode == STATION);
  }

  public Command goToL1() {
    return Commands.defer(
        () -> {
          if (currentScoringMode == L1) return Commands.none();
          else if (currentScoringMode == PROCESSOR) {
            return Commands.sequence(
                    elevator.goToState(ElevatorState.L1), wrist.goToState(WristState.L1))
                .alongWith(Commands.runOnce(() -> currentScoringMode = L1));
          } else {
            return goToStateFromCoral(WristState.L1, ElevatorState.L1)
                .alongWith(Commands.runOnce(() -> currentScoringMode = L1));
          }
        },
        new HashSet<>(Arrays.asList(wrist, elevator)));
  }

  public Command goToProcessor() {
    return Commands.defer(
        () -> {
          if (currentScoringMode == PROCESSOR) return Commands.none();
          else if (currentScoringMode == L1) {
            return Commands.sequence(
                    elevator.goToState(ElevatorState.PROCESSOR),
                    wrist.goToState(WristState.PROCESSOR))
                .alongWith(Commands.runOnce(() -> currentScoringMode = PROCESSOR));
          } else if (currentScoringMode == L2_ALGAE
              || currentScoringMode == L3_ALGAE
              || currentScoringMode == NET) {
            return goToStateFromAlgae(WristState.PROCESSOR, ElevatorState.PROCESSOR)
                .alongWith(Commands.runOnce(() -> currentScoringMode = PROCESSOR));
          } else {
            return goToStateFromCoral(WristState.PROCESSOR, ElevatorState.PROCESSOR)
                .alongWith(Commands.runOnce(() -> currentScoringMode = PROCESSOR));
          }
        },
        new HashSet<>(Arrays.asList(wrist, elevator)));
  }

  public Command goToL2Coral() {
    return Commands.defer(
        () -> {
          if (currentScoringMode == L2_CORAL) return Commands.none();
          else if (currentScoringMode == L2_ALGAE) {
            return Commands.sequence(
                    elevator.goToState(ElevatorState.L2), wrist.goToState(WristState.L2_CORAL))
                .alongWith(Commands.runOnce(() -> currentScoringMode = L2_CORAL));
          } else {
            return goToStateFromCoral(WristState.L2_CORAL, ElevatorState.L2)
                .alongWith(Commands.runOnce(() -> currentScoringMode = L2_CORAL));
          }
        },
        new HashSet<>(Arrays.asList(wrist, elevator)));
  }

  public Command goToL2Algae() {
    return Commands.defer(
        () -> {
          if (currentScoringMode == L2_ALGAE) return Commands.none();
          else if (currentScoringMode == L2_CORAL) {
            return Commands.sequence(
                    elevator.goToState(ElevatorState.L2), wrist.goToState(WristState.L2_ALGAE))
                .alongWith(Commands.runOnce(() -> currentScoringMode = L2_ALGAE));
          } else if (currentScoringMode == L3_ALGAE
              || currentScoringMode == NET
              || currentScoringMode == PROCESSOR) {
            return goToStateFromAlgae(WristState.L2_ALGAE, ElevatorState.L2)
                .alongWith(Commands.runOnce(() -> currentScoringMode = L2_ALGAE));
          } else {
            return goToStateFromCoral(WristState.L2_ALGAE, ElevatorState.L2)
                .alongWith(Commands.runOnce(() -> currentScoringMode = L2_ALGAE));
          }
        },
        new HashSet<>(Arrays.asList(wrist, elevator)));
  }

  public Command goToL3Coral() {
    return Commands.defer(
        () -> {
          if (currentScoringMode == L3_CORAL) return Commands.none();
          else if (currentScoringMode == L3_ALGAE) {
            return Commands.sequence(
                    elevator.goToState(ElevatorState.L3), wrist.goToState(WristState.L3_CORAL))
                .alongWith(Commands.runOnce(() -> currentScoringMode = L3_CORAL));
          } else {
            return goToStateFromCoral(WristState.L3_CORAL, ElevatorState.L3)
                .alongWith(Commands.runOnce(() -> currentScoringMode = L3_CORAL));
          }
        },
        new HashSet<>(Arrays.asList(wrist, elevator)));
  }

  public Command goToL3Algae() {
    return Commands.defer(
        () -> {
          if (currentScoringMode == L3_ALGAE) return Commands.none();
          else if (currentScoringMode == L3_CORAL) {
            return Commands.sequence(
                    elevator.goToState(ElevatorState.L3), wrist.goToState(WristState.L3_ALGAE))
                .alongWith(Commands.runOnce(() -> currentScoringMode = L3_ALGAE));
          } else if (currentScoringMode == NET
              || currentScoringMode == L2_ALGAE
              || currentScoringMode == PROCESSOR) {
            return goToStateFromAlgae(WristState.L3_ALGAE, ElevatorState.L3)
                .alongWith(Commands.runOnce(() -> currentScoringMode = L3_ALGAE));
          } else {
            return goToStateFromCoral(WristState.L3_ALGAE, ElevatorState.L3)
                .alongWith(Commands.runOnce(() -> currentScoringMode = L3_ALGAE));
          }
        },
        new HashSet<>(Arrays.asList(wrist, elevator)));
  }

  public Command goToNet() {
    // return Commands.either(
    //     Commands.none(),
    //     goToStateFromCoral(WristState.NET, ElevatorState.NET)
    //         .alongWith(Commands.runOnce(() -> currentScoringMode = NET)),
    //     () -> currentScoringMode == NET);
    return Commands.defer(
        () -> {
          if (currentScoringMode == NET) {
            return Commands.none();
          } else if (currentScoringMode == L2_ALGAE
              || currentScoringMode == L3_ALGAE
              || currentScoringMode == PROCESSOR) {
            return goToStateFromAlgae(WristState.NET, ElevatorState.NET)
                .alongWith(Commands.runOnce(() -> currentScoringMode = NET));
          } else {
            return goToStateFromCoral(WristState.NET, ElevatorState.NET)
                .alongWith(Commands.runOnce(() -> currentScoringMode = NET));
          }
        },
        new HashSet<>(Arrays.asList(wrist, elevator)));
  }

  public Command goToL4() {
    return Commands.either(
        Commands.none(),
        goToStateFromCoral(WristState.L4, ElevatorState.L4)
            .alongWith(Commands.runOnce(() -> currentScoringMode = L4)),
        () -> currentScoringMode == L4);
  }
}
