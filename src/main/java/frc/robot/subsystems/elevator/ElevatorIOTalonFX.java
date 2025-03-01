package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
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
import frc.robot.Constants.ScoringModeType;
import frc.robot.Robot;

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

  boolean profileToggled = false;

  private final Debouncer mainElevatorMotorDebouncer = new Debouncer(0.5);
  private final Debouncer followerElevatorMotorDebouncer = new Debouncer(0.5);

  private final DynamicMotionMagicVoltage posRequest =
      new DynamicMotionMagicVoltage(
          0, ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY, ELEVATOR_MOTION_MAGIC_ACCELERATION, 0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public ElevatorIOTalonFX() {
    TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
    elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    elevatorConfig.CurrentLimits = ELEVATOR_CURRENT_LIMITS_CONFIGS;

    elevatorConfig.Slot0 =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKP(ELEVATOR_kP)
            .withKI(ELEVATOR_kI)
            .withKD(ELEVATOR_kD)
            .withKG(ELEVATOR_kG)
            .withKS(ELEVATOR_kS)
            .withKV(ELEVATOR_kV)
            .withKA(ELEVATOR_kA);

    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = ELEVATOR_MOTION_MAGIC_ACCELERATION;

    elevatorConfig.SoftwareLimitSwitch = ELEVATOR_SOFT_LIMITS_CONFIGS;

    tryUntilOk(5, () -> mainElevatorMotor.getConfigurator().apply(elevatorConfig, 0.75));
    tryUntilOk(5, () -> followerElevatorMotor.getConfigurator().apply(elevatorConfig, 0.75));

    followerElevatorMotor.setControl(new Follower(MAIN_ELEVATOR_MOTOR_ID, false));

    mainElevatorMotor.setPosition(0);
    followerElevatorMotor.setPosition(0);

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

    inputs.mainConnected = mainElevatorMotorDebouncer.calculate(mainElevatorIsOK == StatusCode.OK);
    inputs.mainPositionRad = mainElevatorMotorPosition.getValue().in(Radians);
    inputs.mainVelocityRadPerSec = mainElevatorMotorVelocity.getValue().in(RadiansPerSecond);
    inputs.mainAppliedVolts = mainElevatorMotorAppliedVoltage.getValue().in(Volts);
    inputs.mainCurrentAmps = mainElevatorMotorCurrent.getValue().in(Amps);

    inputs.followerConnected =
        followerElevatorMotorDebouncer.calculate(followerElevatorIsOK == StatusCode.OK);
    inputs.followerPositionRad = followerElevatorMotorPosition.getValue().in(Radians);
    inputs.followerVelocityRadPerSec =
        followerElevatorMotorVelocity.getValue().in(RadiansPerSecond);
    inputs.followerAppliedVolts = followerElevatorMotorAppliedVoltage.getValue().in(Volts);
    inputs.followerCurrentAmps = followerElevatorMotorCurrent.getValue().in(Amps);

    inputs.positionSetpointRad = posRequest.getPositionMeasure().in(Radians);
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

  @Override
  public void holdPosition() {
    mainElevatorMotor.setControl(
        posRequest.withPosition(mainElevatorMotorPosition.refresh().getValue()));
  }

  @Override
  public void toggleMotorProfile() {
    if (Robot.currentScoringType == ScoringModeType.CORAL) {
      posRequest.Acceleration = ELEVATOR_MOTION_MAGIC_ACCELERATION;
      posRequest.Velocity = ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY;
    } else {
      posRequest.Acceleration = 5.0;
      posRequest.Velocity = 10.0;
    }
  }
}
