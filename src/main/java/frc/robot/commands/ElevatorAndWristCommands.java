package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants.WristState;

public class ElevatorAndWristCommands {
  private final Elevator elevator;
  private final Wrist wrist;

  public ElevatorAndWristCommands(Elevator elevator, Wrist wrist) {
    this.elevator = elevator;
    this.wrist = wrist;
  }

  private Command stowWrist() {
    return wrist.goToState(WristState.ZERO);
  }

  private Command goToState(WristState wristState, ElevatorState elevatorState) {
    return Commands.sequence(
        stowWrist(),
        Commands.waitUntil(wrist.isAtStow),
        elevator.goToState(elevatorState),
        Commands.waitUntil(elevator.isAtTargetState),
        wrist.goToState(wristState));
  }

  public Command goToZero() {
    return goToState(WristState.ZERO, ElevatorState.ZERO);
  }

  public Command goToProcessor() {
    return goToState(WristState.PROCESSOR, ElevatorState.PROCESSOR);
  }

  public Command goToStation() {
    return goToState(WristState.STATION, ElevatorState.PROCESSOR);
  }

  public Command goToL1() {
    return goToState(WristState.L1, ElevatorState.L1);
  }

  public Command goToL2() {
    return goToState(WristState.L2, ElevatorState.L2);
  }

  public Command goToL3() {
    return goToState(WristState.L3, ElevatorState.L3);
  }

  public Command goToL4() {
    return goToState(WristState.L1, ElevatorState.L4);
  }

  public Command goToNet() {
    return goToState(WristState.NET, ElevatorState.NET);
  }
}
