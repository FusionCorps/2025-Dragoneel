package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {
  TalonFX mainElevatorMotor = new TalonFX(13);
  TalonFX followerElevatorMotor = new TalonFX(14);

  StatusSignal<Angle> mainElevatorMotorPosition;
  StatusSignal<AngularVelocity> mainElevatorMotorVelocity;
  StatusSignal<Voltage> mainElevatorMotorAppliedVoltage;
  StatusSignal<Current> mainElevatorMotorCurrent;

  StatusSignal<Angle> followerElevatorMotorPosition;
  StatusSignal<AngularVelocity> followerElevatorMotorVelocity;
  StatusSignal<Voltage> followerElevatorMotorAppliedVoltage;
  StatusSignal<Current> followerElevatorMotorCurrent;

  Debouncer mainElevatorMotorDebouncer = new Debouncer(0.5);
  Debouncer followerElevatorMotorDebouncer = new Debouncer(0.5);

  MotionMagicVoltage posRequest = new MotionMagicVoltage(0);

  public ElevatorIOTalonFX() {
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 80;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 70;
    elevatorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    elevatorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;

    // TODO: these need to change
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = 0;
    elevatorConfig.MotionMagic.MotionMagicJerk = 0;

    // TODO: these need to be tuned
    elevatorConfig.Slot0 =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKV(0)
            .withKA(0);

    // TODO: these may need to change
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 100; // rotations
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0; // rotations

    tryUntilOk(5, () -> mainElevatorMotor.getConfigurator().apply(elevatorConfig, 0.25));
    tryUntilOk(5, () -> followerElevatorMotor.getConfigurator().apply(elevatorConfig, 0.25));

    // TODO: this might change
    followerElevatorMotor.setControl(new Follower(13, true));

    mainElevatorMotorPosition = mainElevatorMotor.getPosition();
    mainElevatorMotorVelocity = mainElevatorMotor.getVelocity();
    mainElevatorMotorAppliedVoltage = mainElevatorMotor.getMotorVoltage();
    mainElevatorMotorCurrent = mainElevatorMotor.getStatorCurrent();

    followerElevatorMotorPosition = followerElevatorMotor.getPosition();
    followerElevatorMotorVelocity = followerElevatorMotor.getVelocity();
    followerElevatorMotorAppliedVoltage = followerElevatorMotor.getMotorVoltage();
    followerElevatorMotorCurrent = followerElevatorMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        mainElevatorMotorPosition,
        mainElevatorMotorVelocity,
        mainElevatorMotorAppliedVoltage,
        mainElevatorMotorCurrent,
        followerElevatorMotorPosition,
        followerElevatorMotorVelocity,
        followerElevatorMotorAppliedVoltage,
        followerElevatorMotorCurrent);
    ParentDevice.optimizeBusUtilizationForAll(mainElevatorMotor, followerElevatorMotor);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    StatusCode mainElevatorIsOK =
        BaseStatusSignal.refreshAll(
            mainElevatorMotorPosition,
            mainElevatorMotorVelocity,
            mainElevatorMotorAppliedVoltage,
            mainElevatorMotorCurrent);

    StatusCode followerElevatorIsOK =
        BaseStatusSignal.refreshAll(
            followerElevatorMotorPosition,
            followerElevatorMotorVelocity,
            followerElevatorMotorAppliedVoltage,
            followerElevatorMotorCurrent);

    inputs.mainElevatorMotorConnected =
        mainElevatorMotorDebouncer.calculate(mainElevatorIsOK == StatusCode.OK);
    inputs.mainElevatorPositionRad = mainElevatorMotorPosition.getValue().in(Radians);
    inputs.mainElevatorVelocityRadPerSec =
        mainElevatorMotorVelocity.getValue().in(RadiansPerSecond);
    inputs.mainElevatorAppliedVolts = mainElevatorMotorAppliedVoltage.getValue().in(Volts);
    inputs.mainElevatorCurrentAmps = mainElevatorMotorCurrent.getValue().in(Amps);

    inputs.followerElevatorMotorConnected =
        followerElevatorMotorDebouncer.calculate(followerElevatorIsOK == StatusCode.OK);
    inputs.followerElevatorPositionRad = followerElevatorMotorPosition.getValue().in(Radians);
    inputs.followerElevatorVelocityRadPerSec =
        followerElevatorMotorVelocity.getValue().in(RadiansPerSecond);
    inputs.followerElevatorAppliedVolts = followerElevatorMotorAppliedVoltage.getValue().in(Volts);
    inputs.followerElevatorCurrentAmps = followerElevatorMotorCurrent.getValue().in(Amps);

    inputs.elevatorPositionSetpointRad = posRequest.getPositionMeasure().in(Radians);
  }

  @Override
  public void setPosition(Angle motorTargetRotations) {
    mainElevatorMotor.setControl(posRequest.withPosition(motorTargetRotations.in(Rotations)));
  }
}
