package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_GEAR_RATIO;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_SHAFT_DIAMETER;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {
  /* IO and hardware inputs */
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  /* Connection Alerts */
  private final Alert mainMotorConnectedAlert =
      new Alert("Main Elevator Motor Disconnected.", AlertType.kError);
  private final Alert followerMotorConnectedAlert =
      new Alert("Follower Elevator Motor Disconnected.", AlertType.kError);

  /* State tracker for current height of the elevator */
  @AutoLogOutput private ElevatorState currentElevatorState = ElevatorState.ZERO;

  /* Visualization mechanism for elevator */
  @AutoLogOutput private final LoggedMechanism2d elevatorMechanism = new LoggedMechanism2d(1, 10);
  private final LoggedMechanismRoot2d elevatorHeightIndicatorMover =
      elevatorMechanism.getRoot("Elevator", 0, 0);
  private final LoggedMechanismLigament2d elevatorHeightIndicator =
      elevatorHeightIndicatorMover.append(new LoggedMechanismLigament2d("elevatorIndicator", 1, 0));

  /* Constructor */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  /* Periodically running code */
  @Override
  public void periodic() {
    io.setTargetPosition(currentElevatorState.rotations);
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    elevatorHeightIndicatorMover.setPosition(
        0,
        // rev * circumference/rev / gear ratio = height in meters
        Units.radiansToRotations(inputs.mainElevatorPositionRad)
            * (Math.PI * ELEVATOR_SHAFT_DIAMETER.in(Meters))
            / (ELEVATOR_GEAR_RATIO));

    if (!inputs.mainElevatorMotorConnected) {
      mainMotorConnectedAlert.set(true);
    }

    if (!inputs.followerElevatorMotorConnected) {
      followerMotorConnectedAlert.set(true);
    }

    // driver variables to visualize the elevator state
    SmartDashboard.putBoolean("Elevator HOMED", currentElevatorState == ElevatorState.ZERO);
    SmartDashboard.putBoolean("Elevator L1", currentElevatorState == ElevatorState.L1);
    SmartDashboard.putBoolean("Elevator L2", currentElevatorState == ElevatorState.L2);
    SmartDashboard.putBoolean("Elevator at STATION", currentElevatorState == ElevatorState.STATION);
    SmartDashboard.putBoolean("Elevator L3", currentElevatorState == ElevatorState.L3);
    SmartDashboard.putBoolean("Elevator L4", currentElevatorState == ElevatorState.L4);
    SmartDashboard.putBoolean("Elevator NET", currentElevatorState == ElevatorState.NET);
  }

  public Command goToL1() {
    return this.runOnce(() -> currentElevatorState = ElevatorState.L1).withName("ElevatorL1");
  }

  public Command goToL2() {
    return this.runOnce(() -> currentElevatorState = ElevatorState.L2).withName("ElevatorL2");
  }

  public Command goToStation() {
    return this.runOnce(() -> currentElevatorState = ElevatorState.STATION)
        .withName("ElevatorStation");
  }

  public Command goToL3() {
    return this.runOnce(() -> currentElevatorState = ElevatorState.L3).withName("ElevatorL3");
  }

  public Command goToL4() {
    return this.runOnce(() -> currentElevatorState = ElevatorState.L4).withName("ElevatorL4");
  }

  public Command goToNet() {
    return this.runOnce(() -> currentElevatorState = ElevatorState.NET).withName("ElevatorNet");
  }

  public Command goToZero() {
    return this.runOnce(() -> currentElevatorState = ElevatorState.ZERO).withName("ElevatorZero");
  }

  /**
   * This routine should be called when robot is first enabled. It will slowly lower the elevator
   * until a current spike is detected, then stop the motor and zero its position. This does not
   * require any external sensors.
   */
  public Command runHomingRoutine() {
    return new FunctionalCommand(
            () -> {},
            () -> io.setVoltage(Volts.of(0.1 * -12.0)),
            (interrupted) -> {
              io.setVoltage(Volts.zero());
              currentElevatorState = ElevatorState.ZERO;
              io.zeroPosition();
            },
            () -> inputs.mainElevatorCurrentAmps > 60.0,
            this)
        .withName("ElevatorHomingRouting");
  }
}
