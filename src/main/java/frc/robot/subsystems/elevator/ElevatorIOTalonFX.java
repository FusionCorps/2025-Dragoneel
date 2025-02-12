package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
   final TalonFX mainElevatorMotor = new TalonFX(MAIN_ELEVATOR_MOTOR_ID);
   final TalonFX followerElevatorMotor = new TalonFX(FOLLOWER_ELEVATOR_MOTOR_ID);

  private final StatusSignal<Angle> mainElevatorMotorPosition;
  private final StatusSignal<AngularVelocity> mainElevatorMotorVelocity;
  private final StatusSignal<Voltage> mainElevatorMotorAppliedVoltage;
  private final StatusSignal<Current> mainElevatorMotorCurrent;

  private final StatusSignal<Angle> followerElevatorMotorPosition;
  private final StatusSignal<AngularVelocity> followerElevatorMotorVelocity;
  private final StatusSignal<Voltage> followerElevatorMotorAppliedVoltage;
  private final StatusSignal<Current> followerElevatorMotorCurrent;

  private final Debouncer mainElevatorMotorDebouncer = new Debouncer(0.5);
  private final Debouncer followerElevatorMotorDebouncer = new Debouncer(0.5);

  private final MotionMagicVoltage posRequest = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public ElevatorIOTalonFX() {
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.StatorCurrentLimit = 80;
    elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorConfig.CurrentLimits.SupplyCurrentLimit = 70;
    elevatorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    elevatorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;

    elevatorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    elevatorConfig.Slot0.kP = 2.5;
    elevatorConfig.Slot0.kI = 0.0; // unneeded
    elevatorConfig.Slot0.kD = 0.3;
    elevatorConfig.Slot0.kS = 0.0; // unneeded
    elevatorConfig.Slot0.kG = 0.325;
    elevatorConfig.Slot0.kV = 0.20;
    elevatorConfig.Slot0.kA = 0.0; // unneeded

    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 200;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = 80;
    // elevatorConfig.MotionMagic.MotionMagicJerk = 0;

    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        26.260565610162730401865820956465;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    tryUntilOk(5, () -> mainElevatorMotor.getConfigurator().apply(elevatorConfig, 0.5));
    tryUntilOk(5, () -> followerElevatorMotor.getConfigurator().apply(elevatorConfig, 0.5));

    followerElevatorMotor.setControl(new Follower(MAIN_ELEVATOR_MOTOR_ID, false));

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
  public void setTargetPosition(Angle motorTargetRotations) {
    mainElevatorMotor.setControl(posRequest.withPosition(motorTargetRotations.in(Rotations)));
  }

  @Override
  public void setVoltageOpenLoop(Voltage volts) {
    mainElevatorMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void zeroPosition() {
    mainElevatorMotor.setPosition(0.0);
    followerElevatorMotor.setPosition(0.0);
  }
}
