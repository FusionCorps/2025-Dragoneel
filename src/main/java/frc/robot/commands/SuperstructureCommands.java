package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

// TODO: find a better name for this class
public class SuperstructureCommands {
  private final Elevator elevator;
  private final Wrist wrist;

  public SuperstructureCommands(Elevator elevator, Wrist wrist) {
    this.elevator = elevator;
    this.wrist = wrist;
  }

  public Command goToZero() {
    return elevator.goToZero().alongWith(wrist.goToZero());
  }

  public Command goToProcessor() {
    return elevator.goToProcessor().alongWith(wrist.goToProcessor());
  }

  public Command goToL1() {
    return elevator.goToL1().alongWith(wrist.goToL1());
  }

  public Command goToL2() {
    return elevator.goToL2().alongWith(wrist.goToL2());
  }

  public Command goToStation() {
    return elevator.goToStation().alongWith(wrist.goToStation());
  }

  public Command goToL3() {
    return elevator.goToL3().alongWith(wrist.goToL3());
  }

  public Command goToL4() {
    return elevator.goToL4().alongWith(wrist.goToL4());
  }

  public Command goToNet() {
    return elevator.goToNet().alongWith(wrist.goToNet());
  }
}
