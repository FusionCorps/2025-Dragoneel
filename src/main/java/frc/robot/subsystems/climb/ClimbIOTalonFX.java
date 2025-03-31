package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climb.ClimbConstants.CLIMB_MOTOR_ID;
import static frc.robot.subsystems.climb.ClimbConstants.CLIMB_RETRACT_PCT;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ClimbIOTalonFX implements ClimbIO {
  public final TalonFX climbMotor;

  private final Debouncer climbDebounce = new Debouncer(0.5);
  private final StatusSignal<Angle> climbPosition;
  private final StatusSignal<AngularVelocity> climbVelocity;
  private final StatusSignal<Voltage> climbAppliedVolts;
  private final StatusSignal<Current> climbCurrent;

  VoltageOut voltageOut = new VoltageOut(0);

  public ClimbIOTalonFX() {
    climbMotor = new TalonFX(CLIMB_MOTOR_ID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.MotionMagic.MotionMagicCruiseVelocity = 20;
    config.MotionMagic.MotionMagicAcceleration = 20;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 48;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 48;

    config.Slot0.kS = 0.5;
    config.Slot0.kV = 2.0;
    config.Slot0.kP = 3.0;

    /* Apply configs */
    tryUntilOk(5, () -> climbMotor.getConfigurator().apply(config, 0.75));

    climbPosition = climbMotor.getPosition();
    climbVelocity = climbMotor.getVelocity();
    climbAppliedVolts = climbMotor.getMotorVoltage();
    climbCurrent = climbMotor.getStatorCurrent();

    climbMotor.setPosition(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, climbPosition, climbVelocity, climbAppliedVolts, climbCurrent);
    climbMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    BaseStatusSignal.refreshAll(climbPosition, climbVelocity, climbAppliedVolts, climbCurrent);

    inputs.connected = climbDebounce.calculate(climbMotor.isConnected());
    inputs.positionRad = climbPosition.getValue().in(Radians);
    inputs.velocityRadPerSec = climbVelocity.getValue().in(RadiansPerSecond);
    inputs.appliedVolts = climbAppliedVolts.getValue().in(Volts);
    inputs.currentAmps = climbCurrent.getValue().in(Amps);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    climbMotor.setControl(
        voltageOut.withOutput(voltage).withLimitForwardMotion(false).withLimitReverseMotion(false));
  }

  @Override
  public void retract() {
    climbMotor.setControl(
        voltageOut
            .withOutput(Volts.of(12).times(CLIMB_RETRACT_PCT.get()))
            .withLimitReverseMotion(
                MathUtil.isNear(
                    ClimbConstants.holdPosition.get(),
                    climbPosition.refresh().getValueAsDouble(),
                    3))
            .withLimitForwardMotion(false));
  }

  @Override
  public void zero() {
    climbMotor.setControl(voltageOut.withOutput(0).withLimitReverseMotion(true));
  }

  @Override
  public void setBrake() {
    climbMotor.setNeutralMode(NeutralModeValue.Brake);
    System.out.println("setting climb to brake");
  }

  @Override
  public void setCoast() {
    climbMotor.setNeutralMode(NeutralModeValue.Coast);
    System.out.println("setting climb to coast");
  }

  @Override
  public void zeroPosition() {
    climbMotor.setPosition(0);
    System.out.println("zeroing climb");
  }
}
