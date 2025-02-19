package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.wrist.WristConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;

public class WristIOSparkFlex implements WristIO {
  /* Main scoring motor */
  public final SparkFlex wristMotor;

  /* Encoder */
  private final RelativeEncoder wristMotorEncoder;
  private final AbsoluteEncoder wristMotorAbsoluteEncoder;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.2);
  // ProfiledPIDController pid =
  //     new ProfiledPIDController(0.4, 0, 0, new TrapezoidProfile.Constraints(5, 5));

  SparkClosedLoopController pid;

  private double setpoint = 0.0;

  /* Debounce */
  private final Debouncer wristMotorDebouncer = new Debouncer(0.5);

  public WristIOSparkFlex() {
    wristMotor = new SparkFlex(WRIST_MOTOR_ID, MotorType.kBrushless);
    wristMotorEncoder = wristMotor.getEncoder();
    wristMotorAbsoluteEncoder = wristMotor.getAbsoluteEncoder();

    // pid.enableContinuousInput(-0.5, 0.5);
    pid = wristMotor.getClosedLoopController();

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
        absPosition -> inputs.wristAbsPositionRad = Units.rotationsToRadians(absPosition));
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
    inputs.wristSetpointRad = setpoint;
  }

  @Override
  public void setVoltageOpenLoop(Voltage voltage) {
    /* Set voltage and hope it works */
    wristMotor.setVoltage(voltage);
  }

  @Override
  public void setTargetPosition(Angle angle) {
    setpoint = angle.in(Rotations);
    pid.setReference(
        setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0, ArbFFUnits.kVoltage);

    // pid.setGoal(setpoint);
    // wristMotor.setVoltage(
    //     feedforward.calculate(pid.getSetpoint().position - 0.5, pid.getSetpoint().velocity)
    //         + pid.calculate(wristMotorAbsoluteEncoder.getPosition()));
  }
}
