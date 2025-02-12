package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class WristIOSim implements WristIO {
  private final DCMotorSim wristMotorSim;

  Voltage volts = Volts.zero();

  public WristIOSim() {
    wristMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.0001, 1.0),
            DCMotor.getNeoVortex(1));
  }

  @Override
  public void setTargetPosition(Angle angle) {
    return;
  }

  @Override
  public void setVoltage(Voltage voltage) {
    volts = voltage;
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    wristMotorSim.setInputVoltage(volts.in(Volts));
    wristMotorSim.update(0.02);

    /* Update inputs */
    inputs.wristMotorConnected = true;
    inputs.wristPositionRad = wristMotorSim.getAngularPositionRad();
    inputs.wristVelocityRadPerSec = wristMotorSim.getAngularVelocityRadPerSec();
    inputs.wristAppliedVolts = volts.in(Volts);
    inputs.wristCurrentAmps = wristMotorSim.getCurrentDrawAmps();
  }
}
