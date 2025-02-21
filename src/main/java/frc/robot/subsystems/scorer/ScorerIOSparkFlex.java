package frc.robot.subsystems.scorer;

import static frc.robot.subsystems.scorer.ScorerConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;

public class ScorerIOSparkFlex implements ScorerIO {
  /* Main scoring motor */
  private final SparkFlex scorerMotor;

  /* Encoder */
  private final RelativeEncoder scorerMotorEncoder;

  /* Debounce */
  private final Debouncer scorerMotorDebouncer = new Debouncer(0.5);

  public ScorerIOSparkFlex() {
    scorerMotor = new SparkFlex(SCORER_MOTOR_ID, MotorType.kBrushless);
    scorerMotorEncoder = scorerMotor.getEncoder();

    /* Try to apply */
    tryUntilOk(
        scorerMotor,
        5,
        () ->
            scorerMotor.configure(
                SCORER_CONFIG, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ScorerIOInputs inputs) {
    /* Update inputs */
    sparkStickyFault = false;
    ifOk(
        scorerMotor,
        scorerMotorEncoder::getPosition,
        position -> inputs.positionRad = position);
    ifOk(
        scorerMotor,
        scorerMotorEncoder::getVelocity,
        velocity -> inputs.velocityRadPerSec = velocity);
    ifOk(
        scorerMotor,
        new DoubleSupplier[] {scorerMotor::getAppliedOutput, scorerMotor::getBusVoltage},
        (doubles) -> inputs.appliedVolts = doubles[0] * doubles[1]);
    ifOk(scorerMotor, scorerMotor::getOutputCurrent, current -> inputs.currentAmps = current);

    inputs.connected = scorerMotorDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    /* Set voltage and hope it works */
    scorerMotor.setVoltage(voltage);
  }
}
