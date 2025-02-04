package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.elevatorGearRatio;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
  // private final DCMotorSim elevatorSim =
  //     new DCMotorSim(
  //         LinearSystemId.createElevatorSystem(
  //             DCMotor.getKrakenX60Foc(2),
  //             5.0,
  //             Units.inchesToMeters(elevatorShaftRadiusInches),
  //             elevatorGearRatio),
  //         DCMotor.getKrakenX60Foc(2));

  private final DCMotorSim elevatorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(2), 0.01, elevatorGearRatio),
          DCMotor.getKrakenX60Foc(2));

  private final ProfiledPIDController elevatorPIDController =
      new ProfiledPIDController(
          0.5, 0, 0.01, new TrapezoidProfile.Constraints(5, 10)); // in rotations units

  private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0, 0.0001, 0.5);

  private double appliedVolts;

  public ElevatorIOSim() {
    elevatorPIDController.reset(0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    appliedVolts =
        MathUtil.clamp(
            elevatorPIDController.calculate(elevatorSim.getAngularPositionRotations())
                + elevatorFeedforward.calculate(elevatorPIDController.getSetpoint().velocity),
            -12.0,
            12.0);

    elevatorSim.setInputVoltage(appliedVolts);
    elevatorSim.update(0.02);

    inputs.mainElevatorMotorConnected = true;
    inputs.mainElevatorPositionRad = elevatorSim.getAngularPositionRad();
    inputs.mainElevatorVelocityRadPerSec = elevatorSim.getAngularVelocityRadPerSec();
    inputs.mainElevatorAppliedVolts = appliedVolts;
    inputs.mainElevatorCurrentAmps = elevatorSim.getCurrentDrawAmps();

    inputs.followerElevatorMotorConnected = true;
    inputs.followerElevatorPositionRad = elevatorSim.getAngularPositionRad();
    inputs.followerElevatorVelocityRadPerSec = elevatorSim.getAngularVelocityRadPerSec();
    inputs.followerElevatorAppliedVolts = appliedVolts;
    inputs.followerElevatorCurrentAmps = elevatorSim.getCurrentDrawAmps();

    inputs.elevatorPositionSetpointRad =
        Units.rotationsToRadians(elevatorPIDController.getSetpoint().position);
  }

  @Override
  public void setTargetPosition(Angle motorTargetRotations) {
    elevatorPIDController.setGoal(motorTargetRotations.in(Rotations));
  }

  @Override
  public void setVoltage(Voltage volts) {
    appliedVolts = volts.in(Volts);
  }

  @Override
  public void zeroPosition() {
    elevatorSim.setAngle(0);
    elevatorPIDController.reset(0);
  }
}
