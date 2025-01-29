package frc.robot.subsystems.scorer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScorerConstants.ScorerState;

public class Scorer extends SubsystemBase {
    /* I/O */
    private final ScorerIO io;

    /* Inputs */
    private final ScorerIOInputsAutoLogged inputs = new ScorerIOInputsAutoLogged();

    /* Motor disconnected alert */
    private final Alert motorDisconnectedAlert = new Alert("Scorer motor disconnnected", AlertType.kError);

    @AutoLogOutput
    ScorerState currentScorerState = ScorerState.IDLE;

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

}
