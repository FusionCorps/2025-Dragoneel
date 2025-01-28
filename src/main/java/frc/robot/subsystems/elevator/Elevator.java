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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert mainMotorConnectedAlert =
      new Alert("Main Elevator Motor Disconnected.", AlertType.kError);
  private final Alert followerMotorConnectedAlert =
      new Alert("Follower Elevator Motor Disconnected.", AlertType.kError);

  @AutoLogOutput private ElevatorState currentElevatorState = ElevatorState.STOW;

  @AutoLogOutput private final LoggedMechanism2d elevatorMechanism = new LoggedMechanism2d(1, 72);
  private final LoggedMechanismRoot2d elevatorHeightIndicatorMover =
      elevatorMechanism.getRoot("Elevator", 0, 0);
  private final LoggedMechanismLigament2d elevatorHeightIndicator =
      elevatorHeightIndicatorMover.append(
          new LoggedMechanismLigament2d("elevatorIndicator", 2, 90));

  // driver dashboard helper variables
  boolean isElevatorStowed = true;
  boolean isElevatorL1 = false;
  boolean isElevatorL2 = false;
  boolean isElevatorL3 = false;
  boolean isElevatorL4 = false;
  boolean isElevatorNET = false;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

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

    isElevatorStowed = currentElevatorState == ElevatorState.STOW;
    isElevatorL1 = currentElevatorState == ElevatorState.L1;
    isElevatorL2 = currentElevatorState == ElevatorState.L2;
    isElevatorL3 = currentElevatorState == ElevatorState.L3;
    isElevatorL4 = currentElevatorState == ElevatorState.L4;
    isElevatorNET = currentElevatorState == ElevatorState.NET;

    SmartDashboard.putBoolean("Elevator Stowed", isElevatorStowed);
    SmartDashboard.putBoolean("Elevator L1", isElevatorL1);
    SmartDashboard.putBoolean("Elevator L2", isElevatorL2);
    SmartDashboard.putBoolean("Elevator L3", isElevatorL3);
    SmartDashboard.putBoolean("Elevator L4", isElevatorL4);
    SmartDashboard.putBoolean("Elevator NET", isElevatorNET);
  }

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
            case L3:
              currentElevatorState = ElevatorState.L2;
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
