package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ScorerConstants.ScorerState;
import frc.robot.subsystems.scorer.Scorer;

public class ScorerCommands {
    private ScorerCommands() { }

    public static Command scorerIntakeAlgae(Scorer scorer) {
        return Commands.run(() -> {
            scorer.setState(ScorerState.INTAKE_ALGAE);
        }).finallyDo(() -> {
            scorer.setState(ScorerState.IDLE);
        });
    }

    public static Command scorerIntakeCoral(Scorer scorer) {
        return Commands.run(() -> {
            scorer.setState(ScorerState.INTAKE_CORAL);
        }).finallyDo(() -> {
            scorer.setState(ScorerState.IDLE);
        });
    }

    public static Command scorerShootCoral(Scorer scorer) {
        return Commands.run(() -> {
            scorer.setState(ScorerState.SHOOT_CORAL);
        }).finallyDo(() -> {
            scorer.setState(ScorerState.IDLE);
        });
    }
}