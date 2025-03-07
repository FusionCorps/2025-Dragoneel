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
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ScoringPieceType;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class WristIOSparkFlex implements WristIO {
  /* Main scoring motor */
  public final SparkFlex wristMotor;

  /* Encoder */
  private final RelativeEncoder wristMotorEncoder;
  private final AbsoluteEncoder wristMotorAbsoluteEncoder;

  private final SparkClosedLoopController pidController;

  private double setpoint = 0.0;

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
                WRIST_CONFIG
                    .apply(WRIST_ABSOLUTE_ENCODER_CONFIG)
                    .apply(WRIST_CLOSED_LOOP_CONFIG.apply(WRIST_MAX_MOTION_CONFIG)),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    /* Update inputs */
    sparkStickyFault = false;
    ifOk(wristMotor, wristMotorEncoder::getPosition, position -> inputs.positionRad = position);
    ifOk(
        wristMotor,
        wristMotorAbsoluteEncoder::getPosition,
        absPosition -> inputs.absolutePositionRad = Units.rotationsToRadians(absPosition));
    ifOk(
        wristMotor,
        wristMotorEncoder::getVelocity,
        velocity -> inputs.velocityRadPerSec = velocity);
    ifOk(
        wristMotor,
        new DoubleSupplier[] {wristMotor::getAppliedOutput, wristMotor::getBusVoltage},
        (doubles) -> inputs.appliedVolts = doubles[0] * doubles[1]);
    ifOk(wristMotor, wristMotor::getOutputCurrent, current -> inputs.currentAmps = current);

    inputs.connected = wristMotorDebouncer.calculate(!sparkStickyFault);
    inputs.wristSetpointRad = Units.rotationsToRadians(setpoint);
  }

  @Override
  public void setVoltageOpenLoop(Voltage voltage) {
    wristMotor.setVoltage(voltage);
  }

  @Override
  public void setTargetPosition(Angle angle, Supplier<ScoringPieceType> scoringModeType) {
    setpoint = angle.in(Rotations);
    if (scoringModeType.get().equals(ScoringPieceType.CORAL)) {
      wristMotor.configureAsync(
          new SparkFlexConfig().apply(new ClosedLoopConfig().p(6.0)),
          ResetMode.kNoResetSafeParameters,
          PersistMode.kPersistParameters);
      pidController.setReference(setpoint, ControlType.kPosition);
    } else {
      wristMotor.configureAsync(
          new SparkFlexConfig().apply(new ClosedLoopConfig().p(2.0)),
          ResetMode.kNoResetSafeParameters,
          PersistMode.kPersistParameters);
      pidController.setReference(setpoint, ControlType.kMAXMotionPositionControl);
    }
  }
}
