package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

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
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class ElevatorIOSim implements ElevatorIO {
  private final DCMotorSim elevatorSim;

  ProfiledPIDController elevatorPIDController =
      new ProfiledPIDController(
          0.8, 0, 0.0, new TrapezoidProfile.Constraints(80, 60)); // in rotations units

  ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.0, 0, 0.65, 0.2);

  private double appliedVolts = 0.0;

  boolean isOpenLoop = false;

  public ElevatorIOSim() {
    elevatorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(2), 0.01, 4.0),
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
              SimulatedBattery.getBatteryVoltage().unaryMinus().in(Volts),
              SimulatedBattery.getBatteryVoltage().in(Volts));
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
