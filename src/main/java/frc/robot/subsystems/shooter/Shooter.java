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
import frc.robot.util.LoggedTunableNumber;
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

  LoggedTunableNumber shooterL4Volts =
      new LoggedTunableNumber("/Shooter/L4Volts", ShooterState.SHOOT_CORAL_L4.volts.in(Volts));
  LoggedTunableNumber shooterL1Volts =
      new LoggedTunableNumber("/Shooter/L1Volts", ShooterState.SHOOT_CORAL_L1.volts.in(Volts));
  LoggedTunableNumber shooterDefaultVolts =
      new LoggedTunableNumber(
          "/Shooter/DefaultVolts", ShooterState.SHOOT_CORAL_DEFAULT.volts.in(Volts));
  LoggedTunableNumber shooterAlgaeVolts =
      new LoggedTunableNumber("/Shooter/AlgaeVolts", ShooterState.SHOOT_ALGAE.volts.in(Volts));

  /* Construction method  */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  /* Periodic method */
  @Override
  public void periodic() {
    io.setVoltage(currentShooterState.volts);

    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    motorDisconnectedAlert.set(!inputs.connected);

    // LoggedTunableNumber.ifChanged(hashCode(), nums -> {});
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
                  } else if (wristStateSupplier.get() == WristState.L3_ALGAE)
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
    return (startEnd(
                () -> setState(ShooterState.SHOOT_CORAL_DEFAULT), () -> setState(ShooterState.IDLE))
            .withTimeout(0.75))
        .repeatedly();
  }

  public Command pulseShooterAutoCmd() {
    // turn shooter on and off quickly repeatedly
    return (startEnd(() -> setState(ShooterState.SHOOT_CORAL_L4), () -> setState(ShooterState.IDLE))
            .withTimeout(0.6))
        .repeatedly();
  }
}
