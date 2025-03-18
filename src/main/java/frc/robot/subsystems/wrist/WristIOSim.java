package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.sim.SparkFlexSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class WristIOSim extends WristIOSparkFlex {
  private final DCMotorSim wristMotorSimModel;

  SparkFlexSim wristMotorSim;

  double appliedVolts = 0.0;
  Angle targetPosition = Rotations.zero();

  boolean isOpenLoop = false;

  public WristIOSim() {
    super();
    wristMotorSimModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.0001, 1.0),
            DCMotor.getNeoVortex(1));

    wristMotorSim = new SparkFlexSim(wristMotor, DCMotor.getNeoVortex(1));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    wristMotorSimModel.setInputVoltage(
        wristMotorSim.getAppliedOutput() * RobotController.getBatteryVoltage());

    wristMotorSimModel.update(0.02);

    wristMotorSim.iterate(
        wristMotorSimModel.getAngularVelocityRPM() / 75.0,
        RobotController.getBatteryVoltage(), // Simulated battery voltage, in Volts
        0.02);

    wristMotorSim
        .getAbsoluteEncoderSim()
        .iterate(wristMotorSimModel.getAngularVelocityRPM() / 75.0, 0.02);

    super.updateInputs(inputs);
  }
}
