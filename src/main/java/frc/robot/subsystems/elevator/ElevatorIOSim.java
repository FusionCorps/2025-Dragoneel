package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
  final DCMotorSim elevatorSim =
      new DCMotorSim(
          LinearSystemId.createElevatorSystem(DCMotor.getKrakenX60Foc(2), 5, 0.015, 1.0),
          DCMotor.getKrakenX60Foc(2));

  final ProfiledPIDController elevatorPIDController =
      new ProfiledPIDController(
          5.0, 0, 0.1, new TrapezoidProfile.Constraints(80, 30)); // in rotations units

  final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0, 0.1, 4.0);

  public ElevatorIOSim() {
    elevatorSim.setAngle(0);
    elevatorPIDController.reset(0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    double appliedVolts =
        MathUtil.clamp(
            elevatorPIDController.calculate(
                elevatorSim.getAngularPositionRotations()
                    + elevatorFeedforward.calculate(elevatorPIDController.getSetpoint().velocity)),
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
  public void setPosition(Angle motorTargetRotations) {
    elevatorPIDController.setGoal(motorTargetRotations.in(Rotations));
  }
}
