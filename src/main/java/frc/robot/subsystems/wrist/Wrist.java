package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants.WristState;
import frc.robot.Robot;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  @AutoLogOutput private WristState currentWristState = WristState.ZERO;

  private final Alert wristMotorConnectedAlert =
      new Alert("Wrist Motor Disconnected.", AlertType.kError);

  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // io.setTargetPosition(currentWristState.rotations);
    io.updateInputs(inputs);

    Robot.componentPoses[2] =
        new Pose3d(0.0, 0.0, Robot.componentPoses[1].getZ(), Rotation3d.kZero)
            .transformBy(
                new Transform3d(
                    new Translation3d(), new Rotation3d(0, inputs.wristPositionRad, 0)));

    Logger.processInputs("Wrist", inputs);
    wristMotorConnectedAlert.set(!inputs.wristMotorConnected);
  }

  public Command moveWristRight() {
    return this.runEnd(
        () -> io.setVoltage(Volts.of(0.001 * 12.0)), () -> io.setVoltage(Volts.zero()));
  }

  public Command moveWristLeft() {
    return this.runEnd(
        () -> io.setVoltage(Volts.of(-0.001 * 12.0)), () -> io.setVoltage(Volts.zero()));
  }

  public Command goToZero() {
    return this.runOnce(() -> currentWristState = WristState.ZERO);
  }

  public Command goToProcessor() {
    return this.runOnce(() -> currentWristState = WristState.PROCESSOR);
  }

  public Command goToL1() {
    return this.runOnce(() -> currentWristState = WristState.L1);
  }

  public Command goToL2() {
    return this.runOnce(() -> currentWristState = WristState.L2_AND_L3);
  }

  public Command goToStation() {
    return this.runOnce(() -> currentWristState = WristState.STATION);
  }

  public Command goToL3() {
    return this.runOnce(() -> currentWristState = WristState.L2_AND_L3);
  }

  public Command goToL4() {
    return this.runOnce(() -> currentWristState = WristState.L4);
  }

  public Command goToNet() {
    return this.runOnce(() -> currentWristState = WristState.NET);
  }
}
