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
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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

  boolean isOpenLoop = false;

  /*
   * Tunable position setpoints for the elevator state enums
   */

  LoggedNetworkNumber elevatorProcessorPosition =
      new LoggedNetworkNumber("/Tuning/Elevator/ProcessorPosition", 0.0);
  LoggedNetworkNumber elevatorL1Position =
      new LoggedNetworkNumber("/Tuning/Elevator/L1Position", 0.0);
  LoggedNetworkNumber elevatorL2Position =
      new LoggedNetworkNumber("/Tuning/Elevator/L2Position", 0.0);
  LoggedNetworkNumber elevatorStationPosition =
      new LoggedNetworkNumber("/Tuning/Elevator/StationPosition", 0.0);
  LoggedNetworkNumber elevatorL3Position =
      new LoggedNetworkNumber("/Tuning/Elevator/L3Position", 0.0);
  LoggedNetworkNumber elevatorL4Position =
      new LoggedNetworkNumber("/Tuning/Elevator/L4Position", 0.0);
  LoggedNetworkNumber elevatorNetPosition =
      new LoggedNetworkNumber("/Tuning/Elevator/NetPosition", 0.0);

  /* Constructor */
  public Elevator(ElevatorIO io) {
    this.io = io;

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Second),
                Volts.of(0.5),
                Seconds.of(5),
                state -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setVoltageOpenLoop(volts), null, this));
  }

  /* Periodically running code */
  @Override
  public void periodic() {
    if (!isOpenLoop) io.setTargetPosition(currentElevatorState.rotations);
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

    /*
     * Update the setpoints for the elevator states if they have been changed
     */

    // ElevatorState.PROCESSOR.rotations = Rotations.of(elevatorProcessorPosition.get());
    // ElevatorState.L1.rotations = Rotations.of(elevatorL1Position.get());
    // ElevatorState.L2.rotations = Rotations.of(elevatorL2Position.get());
    // ElevatorState.STATION.rotations = Rotations.of(elevatorStationPosition.get());
    // ElevatorState.L3.rotations = Rotations.of(elevatorL3Position.get());
    // ElevatorState.L4.rotations = Rotations.of(elevatorL4Position.get());
    // ElevatorState.NET.rotations = Rotations.of(elevatorNetPosition.get());
  }

  public Command goToZero() {
    return this.runOnce(
        () -> {
          currentElevatorState = ElevatorState.ZERO;
          isOpenLoop = false;
        });
  }

  public Command goToProcessor() {
    return this.runOnce(
        () -> {
          currentElevatorState = ElevatorState.PROCESSOR;
          isOpenLoop = false;
        });
  }

  public Command goToL1() {
    return this.runOnce(
        () -> {
          currentElevatorState = ElevatorState.L1;
          isOpenLoop = false;
        });
  }

  public Command goToL2() {
    return this.runOnce(
        () -> {
          currentElevatorState = ElevatorState.L2;
          isOpenLoop = false;
        });
  }

  public Command goToStation() {
    return this.runOnce(
        () -> {
          currentElevatorState = ElevatorState.STATION;
          isOpenLoop = false;
        });
  }

  public Command goToL3() {
    return this.runOnce(
        () -> {
          currentElevatorState = ElevatorState.L3;
          isOpenLoop = false;
        });
  }

  public Command goToL4() {
    return this.runOnce(
        () -> {
          currentElevatorState = ElevatorState.L4;
          isOpenLoop = false;
        });
  }

  public Command goToNet() {
    return this.runOnce(
        () -> {
          currentElevatorState = ElevatorState.NET;
          isOpenLoop = false;
        });
  }

  public Command lowerElevator() {
    return this.runEnd(
        () -> {
          io.setVoltageOpenLoop(Volts.of(-0.3 * 12.0));
          isOpenLoop = true;
        },
        () -> io.setVoltageOpenLoop(Volts.zero()));
  }

  public Command raiseElevator() {
    return this.runEnd(
        () -> {
          io.setVoltageOpenLoop(Volts.of(0.3 * 12.0));
          isOpenLoop = true;
        },
        () -> io.setVoltageOpenLoop(Volts.zero()));
  }

  /**
   * This routine should be called when robot is first enabled. It will slowly lower the elevator
   * until a current spike is detected, then stop the motor and zero its position. This does not
   * require any external sensors.
   */
  public Command runHomingRoutine() {
    return new FunctionalCommand(
            () -> {},
            () -> io.setVoltageOpenLoop(Volts.of(0.1 * -12.0)),
            (interrupted) -> {
              io.setVoltageOpenLoop(Volts.zero());
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
