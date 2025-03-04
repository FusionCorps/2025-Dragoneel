package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.ScoringModeType;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;
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
  }

  public void setState(ShooterState state) {
    currentShooterState = state;
  }

  /** Runs the scorer to outtake algae. */
  public Command shootAlgaeCmd() {
    return startEnd(() -> setState(ShooterState.SHOOT_ALGAE), () -> setState(ShooterState.IDLE));
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
    return Commands.defer(
        () ->
            startEnd(
                () -> {
                  if (currentElevatorStateSupplier.get() == ElevatorState.L1) {
                    setState(ShooterState.SHOOT_CORAL_L1);
                  } else if ((currentElevatorStateSupplier.get() == ElevatorState.L4)) {
                    setState(ShooterState.SHOOT_CORAL_L4);
                  } else if (Robot.currentScoringType == ScoringModeType.ALGAE)
                    setState(ShooterState.SHOOT_ALGAE);
                  else setState(ShooterState.SHOOT_CORAL_DEFAULT);
                },
                () -> setState(ShooterState.IDLE)),
        Set.of(this));
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

  /* Note we only ever shoot L4 in auto. "Pulse" L4 three times. */
  public Command shootCoralInAutoCmd(
      Trigger wristAtScoringState, Supplier<ElevatorState> currentElevatorStateSupplier) {
    return Commands.waitUntil(wristAtScoringState)
        .andThen(
            Commands.sequence(
                runOnce(() -> setState(ShooterState.SHOOT_CORAL_L4)),
                Commands.waitSeconds(0.15),
                runOnce(() -> setState(ShooterState.IDLE)),
                Commands.waitSeconds(0.15),
                runOnce(() -> setState(ShooterState.SHOOT_CORAL_L4)),
                Commands.waitSeconds(0.2),
                runOnce(() -> setState(ShooterState.IDLE)),
                Commands.waitSeconds(0.15),
                runOnce(() -> setState(ShooterState.SHOOT_CORAL_L4)),
                Commands.waitSeconds(0.4),
                runOnce(() -> setState(ShooterState.IDLE))));
  }
}
