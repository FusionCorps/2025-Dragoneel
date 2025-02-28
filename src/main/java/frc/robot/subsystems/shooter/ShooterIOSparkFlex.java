package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;

public class ShooterIOSparkFlex implements ShooterIO {
  /* Main scoring motor */
  private final SparkFlex shooterMotor;

  /* Encoder */
  private final RelativeEncoder shooterMotorEncoder;

  /* Debounce */
  private final Debouncer shooterMotorDebouncer = new Debouncer(0.5);

  public ShooterIOSparkFlex() {
    shooterMotor = new SparkFlex(SHOOTER_MOTOR_ID, MotorType.kBrushless);
    shooterMotorEncoder = shooterMotor.getEncoder();

    /* Try to apply */
    tryUntilOk(
        shooterMotor,
        5,
        () ->
            shooterMotor.configure(
                SHOOTER_CONFIG, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    /* Update inputs */
    sparkStickyFault = false;
    ifOk(shooterMotor, shooterMotorEncoder::getPosition, position -> inputs.positionRad = position);
    ifOk(
        shooterMotor,
        shooterMotorEncoder::getVelocity,
        velocity -> inputs.velocityRadPerSec = velocity);
    ifOk(
        shooterMotor,
        new DoubleSupplier[] {shooterMotor::getAppliedOutput, shooterMotor::getBusVoltage},
        (doubles) -> inputs.appliedVolts = doubles[0] * doubles[1]);
    ifOk(shooterMotor, shooterMotor::getOutputCurrent, current -> inputs.currentAmps = current);

    inputs.connected = shooterMotorDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    /* Set voltage and hope it works */
    shooterMotor.setVoltage(voltage);
  }
}
