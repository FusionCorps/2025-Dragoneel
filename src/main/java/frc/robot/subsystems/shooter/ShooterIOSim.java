package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  /* Motor simulator */
  private final DCMotorSim shooterMotorSim;

  /* Gearbox */
  private final DCMotor shooterMotorGearbox = DCMotor.getNeoVortex(1);

  private Voltage appliedVolts = Volts.of(0.0);

  public ShooterIOSim() {
    shooterMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(shooterMotorGearbox, 0.0001, 1.0),
            shooterMotorGearbox);

    // new PhotonPoseEstimator(null, null, null).
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    shooterMotorSim.setInputVoltage(appliedVolts.in(Volts));
    shooterMotorSim.update(0.02);

    /* Update inputs */
    inputs.connected = true;
    inputs.positionRad = shooterMotorSim.getAngularPositionRad();
    inputs.velocityRadPerSec = shooterMotorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts.in(Volts);
    inputs.currentAmps = shooterMotorSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(Voltage voltage) {
    appliedVolts = voltage;
  }
}
