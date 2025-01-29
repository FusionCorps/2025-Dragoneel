package frc.robot.subsystems.scorer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import static edu.wpi.first.units.Units.Volts;
import frc.robot.Constants.ScorerConstants;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import java.util.function.DoubleSupplier;

public class ScorerIOSparkFlex implements ScorerIO {
    /* Main scoring motor */
    SparkFlex scorerMotor = new SparkFlex(ScorerConstants.ScorerMotorID, MotorType.kBrushless);

    /* Encoder */
    private final RelativeEncoder scorerMotorEncoder = scorerMotor.getEncoder();
    
    /* Debounce */
    private final Debouncer scorerMotorDebouncer = new Debouncer(0.5);

    /* Motor config - TODO: Constantize? Complains about a typecast to SparkFlexConfig */
    SparkFlexConfig config;

    public ScorerIOSparkFlex() {
        /* Setup SparkFlexConfig */
        config = new SparkFlexConfig();
        config
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(RobotController.getBatteryVoltage())
            .smartCurrentLimit(20); // Amps.of(20)..?

        /* Try to apply */
        tryUntilOk(scorerMotor, 5, () -> scorerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    @Override
    public void updateInputs(ScorerIOInputs inputs) {
        /* Update inputs */
        sparkStickyFault = false;
        ifOk(scorerMotor, scorerMotorEncoder::getPosition, position -> inputs.scorerPositionRad = position);
        ifOk(scorerMotor, scorerMotorEncoder::getVelocity, velocity -> inputs.scorerVelocityRadPerSec = velocity);
        ifOk(scorerMotor, new DoubleSupplier[] {scorerMotor::getAppliedOutput, scorerMotor::getBusVoltage}, (doubles) -> inputs.scorerAppliedVolts = doubles[0] * doubles[1]);
        ifOk(scorerMotor, scorerMotor::getOutputCurrent, current -> inputs.scorerCurrentAmps = current);

        inputs.scorerMotorConnected = scorerMotorDebouncer.calculate(!sparkStickyFault); 
    }

    @Override
    public void setVoltage(Voltage voltage) {
        /* Set voltage and hope it works */
        scorerMotor.setVoltage(voltage);
    }
}
