package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climb.ClimbConstants.CLIMB_HOLD_VOLTS;
import static frc.robot.subsystems.climb.ClimbConstants.CLIMB_RUNOUT_VOLTS;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  /* I/O */
  private final ClimbIO io;

  /* Inputs */
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  /* Motor alert */
  private final Alert motorDisconnectedAlert =
      new Alert("Climb Motor Disconnected", AlertType.kError);

  /* Constructor */
  public Climb(ClimbIO io) {
    this.io = io;

    SmartDashboard.putData(
        "Climb/Coast",
        runOnce(() -> io.setCoast())
            .alongWith(Commands.print("climb coast"))
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Climb/Brake",
        runOnce(() -> io.setBrake())
            .alongWith(Commands.print("climb brake"))
            .ignoringDisable(true));
    SmartDashboard.putData(
        "Climb/Zero",
        runOnce(() -> io.zeroPosition())
            .alongWith(Commands.print("climb zero"))
            .ignoringDisable(true));
  }

  /* Periodic */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    motorDisconnectedAlert.set(!inputs.connected);
  }

  /* Run the climb motor and hold position in brake mode when stopped. */
  public Command extendClimbCmd() {
    // Note that the end runnable sets the voltage to 0V, but the neutral mode configuration is set
    // to brake mode.
    return runEnd(() -> io.setVoltage(CLIMB_RUNOUT_VOLTS), () -> io.setVoltage(Volts.zero()));
  }

  public Command retractClimbCmd() {
    // Note that the end runnable sets a little voltage to hold the climb position.
    return runEnd(() -> io.retract(), () -> io.setVoltage(CLIMB_HOLD_VOLTS));
  }
}
