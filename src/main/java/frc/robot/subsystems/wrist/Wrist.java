package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.wrist.WristConstants.WristState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  /* State tracker for current angle of the wrist */
  @AutoLogOutput public WristState currentWristState = WristState.STATION;

  /* Triggers which track when certain states are reached. */
  @AutoLogOutput
  public Trigger isAtStation =
      new Trigger(
          () ->
              Rotations.of(WristState.STATION.rotations.get())
                  .isNear(getCurrentAngle(), Degrees.of(15)));

  @AutoLogOutput
  public Trigger isAtScoringState =
      new Trigger(
          () ->
              Rotations.of(currentWristState.rotations.get())
                      .isNear(getCurrentAngle(), Degrees.of(5))
                  && currentWristState != WristState.STATION);

  private final Alert wristMotorDisconnectedAlert =
      new Alert("Wrist Motor Disconnected.", AlertType.kError);

  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Visualization of the wrist position
    Robot.componentPoses[2] =
        new Pose3d(
            0.135,
            -0.06,
            Robot.componentPoses[1].getZ() + 0.525,
            new Rotation3d(0, -inputs.absolutePositionRad, 0));

    Logger.processInputs("Wrist", inputs);
    wristMotorDisconnectedAlert.set(!inputs.connected);
  }

  /** Set the target state (angle) of the wrist. */
  public Command runTargetState(WristState targetState) {
    return run(
        () -> {
          io.setTargetPosition(
              Rotations.of(targetState.rotations.get()),
              RobotContainer::getCurrentScoringPieceType);
          currentWristState = targetState;
        });
  }

  /** Set the wrist motor to move in "algae speed" mode. */
  public void setToAlgaeSpeed() {
    io.setToAlgaeSpeed();
  }

  /** Set the wrist motor to move in "coral speed" mode. */
  public void setToCoralSpeed() {
    io.setToCoralSpeed();
  }

  public WristState getCurrentWristState() {
    return currentWristState;
  }

  private Angle getCurrentAngle() {
    return Radians.of(inputs.absolutePositionRad);
  }
}
