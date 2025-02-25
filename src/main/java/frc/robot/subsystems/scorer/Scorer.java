package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.scorer.ScorerConstants.ScorerState;
import java.util.function.Supplier;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Scorer extends SubsystemBase {
  /* I/O */
  private final ScorerIO io;

  /* Inputs */
  private final ScorerIOInputsAutoLogged inputs = new ScorerIOInputsAutoLogged();

  /* Motor disconnected alert */
  private final Alert motorDisconnectedAlert =
      new Alert("Scorer Motor Disconnnected", AlertType.kError);

  @AutoLogOutput private ScorerState currentScorerState = ScorerState.IDLE;

  /* Construction method  */
  public Scorer(ScorerIO io) {
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
   * @param elevatorStateSupplier Supplies the current elevator state.
   */
  public Command shootCoralCmd(
      Supplier<ElevatorState> elevatorStateSupplier,
      Supplier<ReefscapeCoralOnFly> coralProjectileSupplier) {
    return this.startEnd(
            () -> {
              if (elevatorStateSupplier.get() == ElevatorState.L1) {
                setState(ScorerState.SHOOT_CORAL_L1);
              } else {
                setState(ScorerState.SHOOT_CORAL_DEFAULT);
              }
              if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
                Arena2025Reefscape.getInstance()
                    .addGamePieceProjectile(coralProjectileSupplier.get());
              }
            },
            () -> setState(ScorerState.IDLE))
        .withTimeout(Seconds.of(2.0));
  }

  public Command shootCoralCmd(Supplier<ElevatorState> elevatorStateSupplier) {
    return this.startEnd(
            () -> {
              if (elevatorStateSupplier.get() == ElevatorState.L1) {
                setState(ScorerState.SHOOT_CORAL_L1);
              } else {
                setState(ScorerState.SHOOT_CORAL_DEFAULT);
              }
            },
            () -> setState(ScorerState.IDLE))
        .withTimeout(Seconds.of(0.25));
  }
}
