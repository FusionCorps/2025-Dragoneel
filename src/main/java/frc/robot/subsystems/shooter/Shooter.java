package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;
import frc.robot.subsystems.wrist.WristConstants.WristState;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert =
      new Alert("Shooter Motor Disconnnected", AlertType.kError);

  /* State tracker for current state of shooter */
  @AutoLogOutput private ShooterState currentShooterState = ShooterState.IDLE;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.setVoltage(Volts.of(currentShooterState.volts.get()));

    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    motorDisconnectedAlert.set(!inputs.connected);
  }

  /** Set the target state (speed) of the shooter. */
  public void setTargetState(ShooterState state) {
    currentShooterState = state;
  }

  /** Runs the scorer to outtake algae. */
  public Command shootAlgaeCmd() {
    return startEnd(
        () -> setTargetState(ShooterState.SHOOT_ALGAE), () -> setTargetState(ShooterState.IDLE));
  }

  /** Run shooter at various speeds depending on current state. */
  public Command shootCoralCmd(Supplier<WristState> wristStateSupplier) {
    return Commands.defer(
        () ->
            startEnd(
                () -> {
                  if (wristStateSupplier.get() == WristState.L1) {
                    setTargetState(ShooterState.SHOOT_CORAL_L1);
                  } else if (wristStateSupplier.get() == WristState.L4) {
                    setTargetState(ShooterState.SHOOT_CORAL_L4);
                  } else if (wristStateSupplier.get() == WristState.L3_ALGAE
                      || wristStateSupplier.get() == WristState.L2_ALGAE)
                    setTargetState(ShooterState.PULL_IN_ALGAE);
                  else setTargetState(ShooterState.SHOOT_CORAL_DEFAULT);
                },
                () -> setTargetState(ShooterState.IDLE)),
        Set.of(this));
  }

  /** Specially used for shooting L4 coral in autonomous period. */
  public Command shootCoralL4InAutoCmd(Trigger wristAtScoringState) {
    return Commands.waitUntil(wristAtScoringState)
        .andThen(
            startEnd(
                () -> setTargetState(ShooterState.SHOOT_CORAL_L4),
                () -> setTargetState(ShooterState.IDLE)));
  }

  /** Pulses shooter repeatedly. Used to hold algae in the physical claw mechanism. */
  public Command pulseShooterCmd() {
    return Commands.sequence(
            runOnce(() -> setTargetState(ShooterState.SHOOT_CORAL_DEFAULT)),
            Commands.waitSeconds(0.6),
            runOnce(() -> setTargetState(ShooterState.IDLE)),
            Commands.waitSeconds(0.2))
        .repeatedly();
  }

  /**
   * Pulses shooter repeatedly. Used to push out coral that doesn't shoot out cleanly in autonomous
   * period.
   */
  public Command pulseShooterAutoCmd() {
    return (startEnd(
                () -> setTargetState(ShooterState.SHOOT_CORAL_L4),
                () -> setTargetState(ShooterState.IDLE))
            .withTimeout(0.6))
        .repeatedly();
  }
}
