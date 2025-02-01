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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX mainElevatorMotor;
  private final TalonFX followerElevatorMotor;

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
    mainElevatorMotor = new TalonFX(mainElevatorMotorID);
    followerElevatorMotor = new TalonFX(followerElevatorMotorID);
    tryUntilOk(5, () -> mainElevatorMotor.getConfigurator().apply(elevatorConfig, 0.5));
    tryUntilOk(5, () -> followerElevatorMotor.getConfigurator().apply(elevatorConfig, 0.5));

    // TODO: this might change
    followerElevatorMotor.setControl(new Follower(mainElevatorMotorID, true));

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

  @Override
  public void setVoltage(Voltage volts) {
    mainElevatorMotor.setControl(voltageRequest.withOutput(volts));
  }
}
