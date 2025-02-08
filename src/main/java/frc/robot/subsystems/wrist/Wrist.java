package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final Alert wristMotorConnectedAlert =
      new Alert("Wrist Motor Disconnected.", AlertType.kError);

  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Wrist", inputs);
    wristMotorConnectedAlert.set(!inputs.WristMotorConnected);
  }

  public Command moveWristRight() {
    return this.runEnd(
        () -> io.setVoltage(Volts.of(0.1 * 12.0)), () -> io.setVoltage(Volts.zero()));
  }

  public Command moveWristLeft() {
    return this.runEnd(
        () -> io.setVoltage(Volts.of(-0.1 * 12.0)), () -> io.setVoltage(Volts.zero()));
  }
}
