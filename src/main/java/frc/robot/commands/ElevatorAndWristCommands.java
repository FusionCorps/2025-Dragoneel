package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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

  public Command goToZero() {
    return elevator.goToState(ElevatorState.ZERO).alongWith(wrist.goToState(WristState.ZERO));
  }

  public Command goToProcessor() {
    return elevator
        .goToState(ElevatorState.PROCESSOR)
        .alongWith(wrist.goToState(WristState.PROCESSOR));
  }

  public Command goToStation() {
    return elevator.goToState(ElevatorState.STATION).alongWith(wrist.goToState(WristState.STATION));
  }

  public Command goToL1() {
    return elevator.goToState(ElevatorState.L1).alongWith(wrist.goToState(WristState.L1));
  }

  public Command goToL2() {
    return elevator.goToState(ElevatorState.L2).alongWith(wrist.goToState(WristState.L2_AND_L3));
  }

  public Command goToL3() {
    return elevator.goToState(ElevatorState.L3).alongWith(wrist.goToState(WristState.L2_AND_L3));
  }

  public Command goToL4() {
    return elevator.goToState(ElevatorState.L4).alongWith(wrist.goToState(WristState.L4));
  }

  public Command goToNet() {
    return elevator.goToState(ElevatorState.NET).alongWith(wrist.goToState(WristState.NET));
  }
}
