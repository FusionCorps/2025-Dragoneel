package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ClimbIOTalonFX implements ClimbIO {
  private final TalonFX climbMotor = new TalonFX(Constants.CLIMB_MOTOR_ID);

  private final Debouncer climbDebounce = new Debouncer(0.5);

  public ClimbIOTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 70;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> climbMotor.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.climbMotorConnected = climbDebounce.calculate(climbMotor.isConnected());
    inputs.climbPositionRad = climbMotor.getPosition().getValue().in(Radians);
    inputs.climbVelocityRadPerSec = climbMotor.getVelocity().getValue().in(RadiansPerSecond);
    inputs.climbAppliedVolts = climbMotor.getMotorVoltage().getValue().in(Volts);
    inputs.climbCurrentAmps = climbMotor.getStatorCurrent().getValue().in(Amps);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    climbMotor.setVoltage(voltage.magnitude());
  }

  @Override
  public void setNeutral() {
    climbMotor.setNeutralMode(NeutralModeValue.Brake);
  }
}
