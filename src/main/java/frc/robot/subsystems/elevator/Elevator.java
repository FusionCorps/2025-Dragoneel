package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_GEAR_RATIO;
import static frc.robot.Constants.ElevatorConstants.ELEVATOR_SHAFT_DIAMETER;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.Robot;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  /* IO and hardware inputs */
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  /* Connection Alerts */
  private final Alert mainMotorConnectedAlert =
      new Alert("Main Elevator Motor Disconnected.", AlertType.kError);
  private final Alert followerMotorConnectedAlert =
      new Alert("Follower Elevator Motor Disconnected.", AlertType.kError);

  /* State tracker for current height of the elevator */
  @AutoLogOutput private ElevatorState currentElevatorState = ElevatorState.ZERO;

  private final SysIdRoutine sysIdRoutine;
  /* Constructor */
  public Elevator(ElevatorIO io) {
    this.io = io;

    sysIdRoutine =  new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(0.25).per(Second), Volts.of(0.5), Seconds.of(5), state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
      new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts), null, this)
    );
  }

  /* Periodically running code */
  @Override
  public void periodic() {
    // io.setTargetPosition(currentElevatorState.rotations);
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    double elevatorStage2HeightMeters =
        // rev * circumference/rev / gear ratio = height in meters
        Units.radiansToRotations(inputs.mainElevatorPositionRad)
            * (Math.PI * ELEVATOR_SHAFT_DIAMETER.in(Meters))
            / (ELEVATOR_GEAR_RATIO);

    double elevatorStage3HeightMeters = elevatorStage2HeightMeters * 2.0;

    Robot.componentPoses[0] = new Pose3d(0.0, 0.0, elevatorStage2HeightMeters, Rotation3d.kZero);
    Robot.componentPoses[1] = new Pose3d(0.0, 0.0, elevatorStage3HeightMeters, Rotation3d.kZero);

    if (!inputs.mainElevatorMotorConnected) {
      mainMotorConnectedAlert.set(true);
    }

    if (!inputs.followerElevatorMotorConnected) {
      followerMotorConnectedAlert.set(true);
    }

    // driver variables to visualize the elevator state
    SmartDashboard.putBoolean("Elevator HOMED", currentElevatorState == ElevatorState.ZERO);
    SmartDashboard.putBoolean("Elevator L1", currentElevatorState == ElevatorState.L1);
    SmartDashboard.putBoolean("Elevator L2", currentElevatorState == ElevatorState.L2);
    SmartDashboard.putBoolean("Elevator at STATION", currentElevatorState == ElevatorState.STATION);
    SmartDashboard.putBoolean("Elevator L3", currentElevatorState == ElevatorState.L3);
    SmartDashboard.putBoolean("Elevator L4", currentElevatorState == ElevatorState.L4);
    SmartDashboard.putBoolean("Elevator NET", currentElevatorState == ElevatorState.NET);
  }

  public Command goToZero() {
    return this.runOnce(() -> currentElevatorState = ElevatorState.ZERO).withName("ElevatorZero");
  }

  public Command goToProcessor() {
    return this.runOnce(() -> currentElevatorState = ElevatorState.PROCESSOR)
        .withName("ElevatorProcessor");
  }

  public Command goToL1() {
    return this.runOnce(() -> currentElevatorState = ElevatorState.L1).withName("ElevatorL1");
  }

  public Command goToL2() {
    return this.runOnce(() -> currentElevatorState = ElevatorState.L2).withName("ElevatorL2");
  }

  public Command goToStation() {
    return this.runOnce(() -> currentElevatorState = ElevatorState.STATION)
        .withName("ElevatorStation");
  }

  public Command goToL3() {
    return this.runOnce(() -> currentElevatorState = ElevatorState.L3).withName("ElevatorL3");
  }

  public Command goToL4() {
    return this.runOnce(() -> currentElevatorState = ElevatorState.L4).withName("ElevatorL4");
  }

  public Command goToNet() {
    return this.runOnce(() -> currentElevatorState = ElevatorState.NET).withName("ElevatorNet");
  }

  public Command lowerElevator() {
    return this.runEnd(
        () -> io.setVoltage(Volts.of(-0.3 * 12.0)), () -> io.setVoltage(Volts.zero()));
  }

  public Command raiseElevator() {
    return this.runEnd(
        () -> io.setVoltage(Volts.of(0.3 * 12.0)), () -> io.setVoltage(Volts.zero()));
  }

  /**
   * This routine should be called when robot is first enabled. It will slowly lower the elevator
   * until a current spike is detected, then stop the motor and zero its position. This does not
   * require any external sensors.
   */
  public Command runHomingRoutine() {
    return new FunctionalCommand(
            () -> {},
            () -> io.setVoltage(Volts.of(0.1 * -12.0)),
            (interrupted) -> {
              io.setVoltage(Volts.zero());
              currentElevatorState = ElevatorState.ZERO;
              io.zeroPosition();
            },
            () -> inputs.mainElevatorCurrentAmps > 60.0,
            this)
        .withName("ElevatorHomingRouting");
  }

  public Command sysIdQuasistatic(Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
