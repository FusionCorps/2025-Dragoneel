package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ScorerIOSim implements ScorerIO {
  /* Motor simulator */
  private final DCMotorSim scorerMotorSim;

  /* Gearbox */
  private final DCMotor scorerMotorGearbox = DCMotor.getNeoVortex(1);

  private Voltage appliedVolts;

  public ScorerIOSim() {
    scorerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(scorerMotorGearbox, 0.0001, 1.0),
            scorerMotorGearbox);
  }

  @Override
  public void updateInputs(ScorerIOInputs inputs) {
    scorerMotorSim.setInputVoltage(appliedVolts.in(Volts));

    /* Update inputs */
    inputs.scorerMotorConnected = true;
    inputs.scorerPositionRad = scorerMotorSim.getAngularPositionRad();
    inputs.scorerVelocityRadPerSec = scorerMotorSim.getAngularVelocityRadPerSec();
    inputs.scorerAppliedVolts = appliedVolts.in(Volts);
    inputs.scorerCurrentAmps = scorerMotorSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(Voltage voltage) {
    appliedVolts = voltage;
  }
}
