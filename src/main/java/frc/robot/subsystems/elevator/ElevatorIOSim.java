package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.ELEVATOR_GEAR_RATIO;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim extends ElevatorIOTalonFX {
  TalonFXSimState mainMotorSim = mainElevatorMotor.getSimState();
  TalonFXSimState followerMotorSim = followerElevatorMotor.getSimState();

  private final DCMotorSim elevatorSim;

  private double appliedVolts = 0.0;

  boolean isOpenLoop = false;

  public ElevatorIOSim() {
    elevatorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(2), 0.0001, ELEVATOR_GEAR_RATIO),
            DCMotor.getKrakenX60Foc(2));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    mainMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    followerMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    // get the motor voltage of the TalonFX
    Voltage motorVoltage = mainMotorSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    elevatorSim.setInputVoltage(motorVoltage.in(Volts));
    elevatorSim.update(0.02); // assume 20 ms loop time

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    mainMotorSim.setRawRotorPosition(elevatorSim.getAngularPosition().times(ELEVATOR_GEAR_RATIO));
    mainMotorSim.setRotorVelocity(elevatorSim.getAngularVelocity().times(ELEVATOR_GEAR_RATIO));

    // do the same for the follower motor
    followerMotorSim.setRawRotorPosition(
        elevatorSim.getAngularPosition().times(ELEVATOR_GEAR_RATIO));
    followerMotorSim.setRotorVelocity(elevatorSim.getAngularVelocity().times(ELEVATOR_GEAR_RATIO));

    super.updateInputs(inputs);
  }
}
