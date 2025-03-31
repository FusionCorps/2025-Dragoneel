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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.wrist.WristConstants.WristState;
import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  public final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  @AutoLogOutput public WristState currentWristState = WristState.STATION;

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

    Robot.componentPoses[2] =
        new Pose3d(
            0.135,
            -0.06,
            Robot.componentPoses[1].getZ() + 0.525,
            new Rotation3d(0, -inputs.absolutePositionRad, 0));

    Logger.processInputs("Wrist", inputs);
    wristMotorDisconnectedAlert.set(!inputs.connected);
  }

  public Command runTargetState(WristState targetState) {
    return run(
        () -> {
          io.setTargetPosition(
              Rotations.of(targetState.rotations.get()),
              RobotContainer::getCurrentScoringPieceType);
          currentWristState = targetState;
        });
  }

  public Command toggleWristSpeed() {
    return Commands.defer(() -> runOnce(() -> io.toggleSpeed()), Set.of(this));
  }

  public void setToAlgaeSpeed() {
    io.setToAlgaeSpeed();
  }

  public void setToCoralSpeed() {
    io.setToCoralSpeed();
  }

  public Angle getCurrentAngle() {
    return Radians.of(inputs.absolutePositionRad);
  }

  public WristState getCurrentWristState() {
    return currentWristState;
  }
}
