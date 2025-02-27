package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.shooter.ShooterConstants.ScorerState;
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
      new Alert("Scorer Motor Disconnnected", AlertType.kError);

  @AutoLogOutput private ScorerState currentScorerState = ScorerState.IDLE;

  /* Construction method  */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  /* Periodic method */
  @Override
  public void periodic() {
    io.setVoltage(currentScorerState.volts);

    io.updateInputs(inputs);
    Logger.processInputs("Scorer", inputs);
    motorDisconnectedAlert.set(!inputs.connected);
  }

  public void setState(ScorerState state) {
    currentScorerState = state;
  }

  /** Runs the scorer to outtake algae. */
  public Command shootAlgaeCmd() {
    return startEnd(() -> setState(ScorerState.SHOOT_ALGAE), () -> setState(ScorerState.IDLE));
  }

  /**
   * Runs the scorer to shoot stored coral. This simultaneously intakes algae. Shoots coral at
   * different speed on L1 vs other levels.
   *
   * @param currentElevatorStateSupplier Supplies the current elevator state.
   */
  public Command shootCoralCmd(
      Supplier<ElevatorState> currentElevatorStateSupplier,
      Supplier<ReefscapeCoralOnFly> coralProjectileSupplier) {
    return shootCoralCmd(currentElevatorStateSupplier)
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

  public Command shootCoralCmd(Supplier<ElevatorState> currentElevatorStateSupplier) {
    return startEnd(
            () -> {
              if (currentElevatorStateSupplier.get() == ElevatorState.L1) {
                setState(ScorerState.SHOOT_CORAL_L1);
              } else if ((currentElevatorStateSupplier.get() == ElevatorState.L4)) {
                setState(ScorerState.SHOOT_CORAL_L1);
              } else setState(ScorerState.SHOOT_CORAL_DEFAULT);
            },
            () -> setState(ScorerState.IDLE))
        .withTimeout(Seconds.of(2.0));
  }

  public Command shootCoralInAutoCmd(
      Trigger wristAtState,
      Supplier<ElevatorState> currentElevatorStateSupplier,
      Supplier<ReefscapeCoralOnFly> coralProjectileSupplier) {
    return shootCoralInAutoCmd(wristAtState, currentElevatorStateSupplier)
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

  public Command shootCoralInAutoCmd(
      Trigger wristAtState, Supplier<ElevatorState> currentElevatorStateSupplier) {
    return Commands.waitUntil(wristAtState)
        .andThen(
            startEnd(
                    () -> {
                      if (currentElevatorStateSupplier.get() == ElevatorState.L1) {
                        setState(ScorerState.SHOOT_CORAL_L1);
                      } else if ((currentElevatorStateSupplier.get() == ElevatorState.L4)) {
                        setState(ScorerState.SHOOT_CORAL_L1);
                      } else setState(ScorerState.SHOOT_CORAL_DEFAULT);
                    },
                    () -> setState(ScorerState.IDLE))
                .withTimeout(Seconds.of(2.0)));
  }
}
