package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim extends ClimbIOTalonFX {
  TalonFXSimState climbMotorSim = climbMotor.getSimState();
  private final DCMotorSim climbSim;
  private final DCMotor climbMotorGearbox = DCMotor.getKrakenX60(1);

  public ClimbIOSim() {
    climbSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(climbMotorGearbox, 0.0001, 1.0), climbMotorGearbox);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    climbMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    Voltage motorVoltage = climbMotorSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    climbSim.setInputVoltage(motorVoltage.in(Volts));
    climbSim.update(0.02); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    climbMotorSim.setRawRotorPosition(climbSim.getAngularPosition());
    climbMotorSim.setRotorVelocity(climbSim.getAngularVelocity());

    super.updateInputs(inputs);
  }
}
