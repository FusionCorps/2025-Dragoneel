package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim implements ClimbIO {
  private final DCMotorSim climbMotorSim;
  private final DCMotor climbMotorGearbox = DCMotor.getKrakenX60(1);
  private Voltage appliedVolts = Volts.of(0);

  public ClimbIOSim() {
    climbMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(climbMotorGearbox, 0.0001, 1.0), climbMotorGearbox);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    climbMotorSim.setInputVoltage(appliedVolts.in(Volts));
    climbMotorSim.update(0.02);

    inputs.climbMotorConnected = true;
    inputs.climbPositionRad = climbMotorSim.getAngularPositionRad();
    inputs.climbVelocityRadPerSec = climbMotorSim.getAngularVelocityRadPerSec();
    inputs.climbAppliedVolts = appliedVolts.in(Volts);
    inputs.climbCurrentAmps = climbMotorSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(Voltage voltage) {
    appliedVolts = voltage;
  }

  @Override
  public void setNeutral() {
    setVoltage(Volts.of(0));
  }
}
