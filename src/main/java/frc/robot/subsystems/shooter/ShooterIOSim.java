package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  /* Motor simulator */
  private final DCMotorSim scorerMotorSim;

  /* Gearbox */
  private final DCMotor scorerMotorGearbox = DCMotor.getNeoVortex(1);

  private Voltage appliedVolts = Volts.of(0.0);

  public ShooterIOSim() {
    scorerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(scorerMotorGearbox, 0.0001, 1.0),
            scorerMotorGearbox);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    scorerMotorSim.setInputVoltage(appliedVolts.in(Volts));
    scorerMotorSim.update(0.02);

    /* Update inputs */
    inputs.connected = true;
    inputs.positionRad = scorerMotorSim.getAngularPositionRad();
    inputs.velocityRadPerSec = scorerMotorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts.in(Volts);
    inputs.currentAmps = scorerMotorSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(Voltage voltage) {
    appliedVolts = voltage;
  }
}
