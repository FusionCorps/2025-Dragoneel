package frc.robot.subsystems.scorer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScorerConstants.ScorerState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Scorer extends SubsystemBase {
  /* I/O */
  private final ScorerIO io;

  /* Inputs */
  private final ScorerIOInputsAutoLogged inputs = new ScorerIOInputsAutoLogged();

  /* Motor disconnected alert */
  private final Alert motorDisconnectedAlert =
      new Alert("Scorer motor disconnnected", AlertType.kError);

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
    motorDisconnectedAlert.set(!inputs.scorerMotorConnected);
  }

  public void setState(ScorerState state) {
    currentScorerState = state;
  }

  /** Runs the scorer to intake algae. This simultaneously shoots stored coral. */
  // TODO: may need to be removed as irrelevant, especially since it is redundant
  public Command intakeAlgaeCmd() {
    return runEnd(() -> setState(ScorerState.INTAKE_ALGAE), () -> setState(ScorerState.IDLE));
  }

  /** Runs the scorer to intake coral from the passive intake. */
  // TODO: may need to be removed as not useful
  public Command intakeCoralCmd() {
    return runEnd(() -> setState(ScorerState.INTAKE_CORAL), () -> setState(ScorerState.IDLE));
  }

  /** Runs the scorer to shoot stored coral. This simultaneously intakes algae. */
  public Command shootCoralCmd() {
    return runEnd(() -> setState(ScorerState.SHOOT_CORAL), () -> setState(ScorerState.IDLE));
  }
}
