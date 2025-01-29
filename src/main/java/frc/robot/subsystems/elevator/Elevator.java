package frc.robot.subsystems.elevator;

import static frc.robot.Constants.ElevatorConstants.elevatorGearRatio;
import static frc.robot.Constants.ElevatorConstants.elevatorShaftRadiusInches;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  @AutoLogOutput private ElevatorState currentElevatorState = ElevatorState.STOW;

  /* Visualization mechanism for elevator */
  @AutoLogOutput private final LoggedMechanism2d elevatorMechanism = new LoggedMechanism2d(1, 72);
  private final LoggedMechanismRoot2d elevatorHeightIndicatorMover =
      elevatorMechanism.getRoot("Elevator", 0, 0);
  private final LoggedMechanismLigament2d elevatorHeightIndicator =
      elevatorHeightIndicatorMover.append(
          new LoggedMechanismLigament2d("elevatorIndicator", 2, 90));

  /* Constructor */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  /* Periodically running code */
  @Override
  public void periodic() {
    io.setPosition(currentElevatorState.height);
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    elevatorHeightIndicatorMover.setPosition(
        0,
        // rev * distance/rev / gear ratio
        Units.radiansToRotations(inputs.mainElevatorPositionRad)
            * (2.0 * Math.PI * elevatorShaftRadiusInches)
            / (elevatorGearRatio));

    if (!inputs.mainElevatorMotorConnected) {
      mainMotorConnectedAlert.set(true);
    }

    if (!inputs.followerElevatorMotorConnected) {
      followerMotorConnectedAlert.set(true);
    }

    // driver variables to visualize the elevator state
    SmartDashboard.putBoolean("Elevator HOMED", currentElevatorState == ElevatorState.STOW);
    SmartDashboard.putBoolean("Elevator L1", currentElevatorState == ElevatorState.L1);
    SmartDashboard.putBoolean("Elevator L2", currentElevatorState == ElevatorState.L2);
    SmartDashboard.putBoolean("Elevator at STATION", currentElevatorState == ElevatorState.STATION);
    SmartDashboard.putBoolean("Elevator L3", currentElevatorState == ElevatorState.L3);
    SmartDashboard.putBoolean("Elevator L4", currentElevatorState == ElevatorState.L4);
    SmartDashboard.putBoolean("Elevator NET", currentElevatorState == ElevatorState.NET);
  }

  /* Command to lower the elevator to the next lowest height */
  public Command lowerElevator() {
    return Commands.runOnce(
        () -> {
          switch (currentElevatorState) {
            case STOW:
              break;
            case L1:
              currentElevatorState = ElevatorState.STOW;
              break;
            case L2:
              currentElevatorState = ElevatorState.L1;
              break;
            case STATION:
              currentElevatorState = ElevatorState.L2;
              break;
            case L3:
              currentElevatorState = ElevatorState.STATION;
              break;
            case L4:
              currentElevatorState = ElevatorState.L3;
              break;
            case NET:
              currentElevatorState = ElevatorState.L4;
              break;
          }
        });
  }

  /* Command to raise the elevator to the next tallest height */
  public Command raiseElevator() {
    return Commands.runOnce(
        () -> {
          switch (currentElevatorState) {
            case STOW:
              currentElevatorState = ElevatorState.L1;
              break;
            case L1:
              currentElevatorState = ElevatorState.L2;
              break;
            case L2:
              currentElevatorState = ElevatorState.STATION;
              break;
            case STATION:
              currentElevatorState = ElevatorState.L3;
              break;
            case L3:
              currentElevatorState = ElevatorState.L4;
              break;
            case L4:
              currentElevatorState = ElevatorState.NET;
              break;
            case NET:
              break;
          }
        });
  }
}
