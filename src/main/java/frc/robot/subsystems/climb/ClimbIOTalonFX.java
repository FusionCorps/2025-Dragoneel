package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.climb.ClimbConstants.CLIMB_MOTOR_ID;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX climbMotor;

  private final Debouncer climbDebounce = new Debouncer(0.5);
  private final StatusSignal<Angle> climbPosition;
  private final StatusSignal<AngularVelocity> climbVelocity;
  private final StatusSignal<Voltage> climbAppliedVolts;
  private final StatusSignal<Current> climbCurrent;

  public ClimbIOTalonFX() {
    climbMotor = new TalonFX(CLIMB_MOTOR_ID);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    /* Apply configs */
    tryUntilOk(5, () -> climbMotor.getConfigurator().apply(config, 0.25));

    climbPosition = climbMotor.getPosition();
    climbVelocity = climbMotor.getVelocity();
    climbAppliedVolts = climbMotor.getMotorVoltage();
    climbCurrent = climbMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, climbPosition, climbVelocity, climbAppliedVolts, climbCurrent);
    climbMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    BaseStatusSignal.refreshAll(climbPosition, climbVelocity, climbAppliedVolts, climbCurrent);

    inputs.climbMotorConnected = climbDebounce.calculate(climbMotor.isConnected());
    inputs.climbPositionRad = climbPosition.getValue().in(Radians);
    inputs.climbVelocityRadPerSec = climbVelocity.getValue().in(RadiansPerSecond);
    inputs.climbAppliedVolts = climbAppliedVolts.getValue().in(Volts);
    inputs.climbCurrentAmps = climbCurrent.getValue().in(Amps);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    climbMotor.setVoltage(voltage.in(Volts));
  }
}
