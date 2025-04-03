package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;
import frc.robot.subsystems.wrist.WristConstants.WristState;
import java.util.Set;
import java.util.function.Supplier;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /* I/O */
  private final ShooterIO io;

  /* Inputs */
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  /* Motor disconnected alert */
  private final Alert motorDisconnectedAlert =
      new Alert("Shooter Motor Disconnnected", AlertType.kError);

  @AutoLogOutput private ShooterState currentShooterState = ShooterState.IDLE;

  boolean pulsing = false;

  /* Construction method  */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  /* Periodic method */
  @Override
  public void periodic() {
    io.setVoltage(Volts.of(currentShooterState.volts.get()));

    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    motorDisconnectedAlert.set(!inputs.connected);
  }

  public void setState(ShooterState state) {
    currentShooterState = state;
  }

  /** Runs the scorer to outtake algae. */
  public Command shootAlgaeCmd() {
    return startEnd(() -> setState(ShooterState.SHOOT_ALGAE), () -> setState(ShooterState.IDLE));
  }

  public Command shootCoralCmd(Supplier<WristState> wristStateSupplier) {
    return Commands.defer(
        () ->
            startEnd(
                () -> {
                  if (wristStateSupplier.get() == WristState.L1) {
                    setState(ShooterState.SHOOT_CORAL_L1);
                  } else if (wristStateSupplier.get() == WristState.L4) {
                    setState(ShooterState.SHOOT_CORAL_L4);
                  } else if (wristStateSupplier.get() == WristState.L3_ALGAE
                      || wristStateSupplier.get() == WristState.L2_ALGAE)
                    setState(ShooterState.PULL_IN_ALGAE);
                  else setState(ShooterState.SHOOT_CORAL_DEFAULT);
                },
                () -> setState(ShooterState.IDLE)),
        Set.of(this));
  }

  /**
   * Runs the scorer to shoot stored coral. This simultaneously intakes algae. Shoots coral at
   * different speed on L1 vs other levels.
   *
   * @param currentElevatorStateSupplier Supplies the current elevator state.
   */
  public Command shootCoralCmd(
      Supplier<WristState> wristState, Supplier<ReefscapeCoralOnFly> coralProjectileSupplier) {
    return shootCoralCmd(wristState)
        .alongWith(
            Commands.runOnce(
                () -> {
                  if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
                    if (coralProjectileSupplier.get() != null) {
                      Arena2025Reefscape.getInstance()
                          .addGamePieceProjectile(coralProjectileSupplier.get());
                    }
                  }
                }));
  }

  public Command shootCoralInAutoCmd(Trigger wristAtScoringState) {
    return Commands.waitUntil(wristAtScoringState)
        .andThen(
            startEnd(
                () -> setState(ShooterState.SHOOT_CORAL_L4), () -> setState(ShooterState.IDLE)));
  }

  public Command shootCoralInAutoCmd(
      Trigger wristAtState, Supplier<ReefscapeCoralOnFly> coralProjectileSupplier) {
    return shootCoralInAutoCmd(wristAtState)
        .alongWith(
            Commands.runOnce(
                () -> {
                  if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
                    if (coralProjectileSupplier.get() != null) {
                      Arena2025Reefscape.getInstance()
                          .addGamePieceProjectile(coralProjectileSupplier.get());
                    }
                  }
                }));
  }

  public Command pulseShooterCmd() {
    // turn shooter on and off quickly repeatedly
    // return (startEnd(
    //             () -> setState(ShooterState.SHOOT_CORAL_DEFAULT), () ->
    // setState(ShooterState.IDLE))
    //         .withTimeout(0.75))
    //     .repeatedly();
    return Commands.sequence(
            runOnce(() -> setState(ShooterState.SHOOT_CORAL_DEFAULT)),
            Commands.waitSeconds(0.6),
            runOnce(() -> setState(ShooterState.IDLE)),
            Commands.waitSeconds(0.2))
        .repeatedly();
  }

  public Command pulseShooterAutoCmd() {
    // turn shooter on and off quickly repeatedly
    return (startEnd(() -> setState(ShooterState.SHOOT_CORAL_L4), () -> setState(ShooterState.IDLE))
            .withTimeout(0.6))
        .repeatedly();
  }
}
