package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.wrist.WristConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;

public class WristIOSparkFlex implements WristIO {
  /* Main scoring motor */
  public final SparkFlex wristMotor;

  /* Encoder */
  private final RelativeEncoder wristMotorEncoder;
  private final AbsoluteEncoder wristMotorAbsoluteEncoder;

  public final SparkClosedLoopController pidController;

  /* Debounce */
  private final Debouncer wristMotorDebouncer = new Debouncer(0.5);

  public WristIOSparkFlex() {
    wristMotor = new SparkFlex(WRIST_MOTOR_ID, MotorType.kBrushless);
    wristMotorEncoder = wristMotor.getEncoder();
    wristMotorAbsoluteEncoder = wristMotor.getAbsoluteEncoder();
    pidController = wristMotor.getClosedLoopController();

    /* Try to apply */
    tryUntilOk(
        wristMotor,
        5,
        () ->
            wristMotor.configure(
                WRIST_CONFIG, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    /* Update inputs */
    sparkStickyFault = false;
    ifOk(
        wristMotor, wristMotorEncoder::getPosition, position -> inputs.wristPositionRad = position);
    ifOk(
        wristMotor,
        wristMotorAbsoluteEncoder::getPosition,
        absPosition -> inputs.wristAbsPositionRad = absPosition);
    ifOk(
        wristMotor,
        wristMotorEncoder::getVelocity,
        velocity -> inputs.wristVelocityRadPerSec = velocity);
    ifOk(
        wristMotor,
        new DoubleSupplier[] {wristMotor::getAppliedOutput, wristMotor::getBusVoltage},
        (doubles) -> inputs.wristAppliedVolts = doubles[0] * doubles[1]);
    ifOk(wristMotor, wristMotor::getOutputCurrent, current -> inputs.wristCurrentAmps = current);

    inputs.wristMotorConnected = wristMotorDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setVoltageOpenLoop(Voltage voltage) {
    /* Set voltage and hope it works */
    wristMotor.setVoltage(voltage);
  }

  @Override
  public void setTargetPosition(Angle angle) {
    pidController.setReference(angle.in(Rotations), ControlType.kPosition);
  }
}
