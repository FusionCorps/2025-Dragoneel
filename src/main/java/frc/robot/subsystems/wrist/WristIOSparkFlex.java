package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.util.SparkUtil.*;
import static frc.robot.Constants.WristConstants.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import java.util.function.DoubleSupplier;

public class WristIOSparkFlex implements WristIO {
  /* Main scoring motor */
  private final SparkFlex wristMotor;

  /* Encoder */
  private final RelativeEncoder wristMotorEncoder;

  private final SparkClosedLoopController pidController;

  /* Debounce */
  private final Debouncer wristMotorDebouncer = new Debouncer(0.5);

  public WristIOSparkFlex() {
    wristMotor = new SparkFlex(WRIST_MOTOR_ID, MotorType.kBrushless);
    wristMotorEncoder = wristMotor.getEncoder();
    pidController = wristMotor.getClosedLoopController();
    SparkFlexConfig cfg = new SparkFlexConfig();
    cfg.inverted(false)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(RobotController.getBatteryVoltage())
        .smartCurrentLimit(60);

    cfg.closedLoop.pid(0, 0, 0);

    /* Try to apply */
    tryUntilOk(
        wristMotor,
        5,
        () ->
            wristMotor.configure(
                cfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    /* Update inputs */
    sparkStickyFault = false;
    ifOk(
        wristMotor, wristMotorEncoder::getPosition, position -> inputs.WristPositionRad = position);
    ifOk(
        wristMotor,
        wristMotorEncoder::getVelocity,
        velocity -> inputs.wristVelocityRadPerSec = velocity);
    ifOk(
        wristMotor,
        new DoubleSupplier[] {wristMotor::getAppliedOutput, wristMotor::getBusVoltage},
        (doubles) -> inputs.WristVoltage = doubles[0] * doubles[1]);
    ifOk(wristMotor, wristMotor::getOutputCurrent, current -> inputs.WristAmps = current);

    inputs.WristMotorConnected = wristMotorDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    /* Set voltage and hope it works */
    wristMotor.setVoltage(voltage);
  }

  @Override
  public void setTargetPosition(Angle angle) {
    pidController.setReference(angle.in(Rotations), ControlType.kPosition);
  }
}
