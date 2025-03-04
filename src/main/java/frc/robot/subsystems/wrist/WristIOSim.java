package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class WristIOSim implements WristIO {
  private final DCMotorSim wristMotorSim;

  ProfiledPIDController controller =
      new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(0, 0));

  double appliedVolts = 0.0;
  Angle targetPosition = Rotations.zero();

  boolean isOpenLoop = false;

  public WristIOSim() {
    wristMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.0001, 1.0),
            DCMotor.getNeoVortex(1));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    if (!isOpenLoop)
      appliedVolts =
          MathUtil.clamp(
              controller.calculate(inputs.positionRad, targetPosition.in(Radians)), -12.0, 12.0);
    wristMotorSim.setInputVoltage(appliedVolts);
    wristMotorSim.update(0.02);

    /* Update inputs */
    inputs.connected = true;
    inputs.positionRad = wristMotorSim.getAngularPositionRad();
    inputs.absolutePositionRad = wristMotorSim.getAngularPositionRad() % (2 * Math.PI);
    inputs.velocityRadPerSec = wristMotorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = wristMotorSim.getCurrentDrawAmps();

    inputs.wristSetpointRad = targetPosition.in(Radians);
  }

  @Override
  public void setTargetPosition(Angle angle) {
    isOpenLoop = false;
    targetPosition = angle;
  }

  @Override
  public void setVoltageOpenLoop(Voltage voltage) {
    isOpenLoop = true;
    appliedVolts = voltage.in(Volts);
  }
}
