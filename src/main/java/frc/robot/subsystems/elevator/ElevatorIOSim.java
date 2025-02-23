package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

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
  private final DCMotorSim elevatorSim;

  ProfiledPIDController elevatorPIDController =
      new ProfiledPIDController(
          1.5, 0, 0.0, new TrapezoidProfile.Constraints(75, 40)); // in rotations units

  ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0, 0, 0.53);

  private double appliedVolts = 0.0;
  private Angle targetPosition = Rotations.zero();

  boolean isOpenLoop = false;

  public ElevatorIOSim() {
    elevatorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(2), 0.01, ELEVATOR_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(2));

    elevatorPIDController.reset(0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (!isOpenLoop) {
      appliedVolts =
          MathUtil.clamp(
              elevatorPIDController.calculate(elevatorSim.getAngularPositionRotations())
                  + elevatorFeedforward.calculate(elevatorPIDController.getSetpoint().velocity),
              -12.0,
              12.0);
    }

    elevatorSim.setInputVoltage(appliedVolts);
    elevatorSim.update(0.02);

    inputs.mainConnected = true;
    inputs.mainPositionRad = elevatorSim.getAngularPositionRad();
    inputs.mainVelocityRadPerSec = elevatorSim.getAngularVelocityRadPerSec();
    inputs.mainAppliedVolts = appliedVolts;
    inputs.mainCurrentAmps = elevatorSim.getCurrentDrawAmps();

    inputs.followerConnected = true;
    inputs.followerPositionRad = elevatorSim.getAngularPositionRad();
    inputs.followerVelocityRadPerSec = elevatorSim.getAngularVelocityRadPerSec();
    inputs.followerAppliedVolts = appliedVolts;
    inputs.followerCurrentAmps = elevatorSim.getCurrentDrawAmps();

    inputs.positionSetpointRad =
        Units.rotationsToRadians(elevatorPIDController.getSetpoint().position);
  }

  @Override
  public void setTargetPosition(Angle motorTargetRotations) {
    isOpenLoop = false;
    targetPosition = motorTargetRotations;
    elevatorPIDController.setGoal(motorTargetRotations.in(Rotations));
  }

  @Override
  public void setVoltageOpenLoop(Voltage volts) {
    isOpenLoop = true;
    appliedVolts = volts.in(Volts);
  }

  @Override
  public void zeroPosition() {
    elevatorSim.setAngle(0);
    elevatorPIDController.reset(0);
  }
}
