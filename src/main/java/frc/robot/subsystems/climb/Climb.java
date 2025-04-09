package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climb.ClimbConstants.CLIMB_HOLD_PCT;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert =
      new Alert("Climb Motor Disconnected", AlertType.kError);

  public Climb(ClimbIO io) {
    this.io = io;

    // Dashboard button to rezero the climb mechanism
    SmartDashboard.putData("Climb/Zero", runOnce(() -> io.zeroPosition()).ignoringDisable(true));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    motorDisconnectedAlert.set(!inputs.connected);
  }

  /** Extend the climb mechanism outward and hold position in brake mode when stopped. */
  public Command extendClimbCmd() {
    // Note that the end runnable sets the voltage to 0V,
    // but the neutral mode config is brake so the motor will hold position.
    return runEnd(() -> io.extend(), () -> io.setVoltage(Volts.zero()));
  }

  /** Retract the climb mechanism inward and hold position in brake mode when stopped. */
  public Command retractClimbCmd() {
    // Note that the end runnable sets a little voltage to hold the climb position.
    return runEnd(
        () -> io.retract(), () -> io.setVoltage(Volts.of(12).times(CLIMB_HOLD_PCT.get())));
  }
}
