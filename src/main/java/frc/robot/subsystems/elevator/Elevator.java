package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  /* IO and hardware inputs */
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  /* Connection Alerts */
  private final Alert mainMotorDisconnectedAlert =
      new Alert("Main Elevator Motor Disconnected.", AlertType.kError);
  private final Alert followerMotorDisconnectedAlert =
      new Alert("Follower Elevator Motor Disconnected.", AlertType.kError);
  private final Alert forwardLimitSwitchTriggeredAlert =
      new Alert("Top Limit Switch Triggered.", AlertType.kInfo);
  private final Alert reverseLimitSwitchTriggeredAlert =
      new Alert("Bottom Limit Switch Triggered. Elevator zeroed", AlertType.kInfo);

  /* State tracker for current height of the elevator */
  private ElevatorState currentElevatorState = ElevatorState.ZERO;

  @AutoLogOutput
  public Trigger isAtTargetState =
      new Trigger(
          () ->
              getCurrentElevatorPosition()
                  .isNear(currentElevatorState.rotations, Rotations.of(0.25)));

  boolean isOpenLoop = false;

  LoggedTunableNumber elevatorProcessorPosition =
      new LoggedTunableNumber(
          "/Elevator/ProcessorPosition", ElevatorState.PROCESSOR.rotations.in(Rotations));
  LoggedTunableNumber elevatorL1Position =
      new LoggedTunableNumber("/Elevator/L1Position", ElevatorState.L1.rotations.in(Rotations));
  LoggedTunableNumber elevatorL2Position =
      new LoggedTunableNumber("/Elevator/L2Position", ElevatorState.L2.rotations.in(Rotations));
  LoggedTunableNumber elevatorStationPosition =
      new LoggedTunableNumber(
          "/Elevator/StationPosition", ElevatorState.STATION.rotations.in(Rotations));
  LoggedTunableNumber elevatorL3Position =
      new LoggedTunableNumber("/Elevator/L3Position", ElevatorState.L3.rotations.in(Rotations));
  LoggedTunableNumber elevatorL4Position =
      new LoggedTunableNumber("/Elevator/L4Position", ElevatorState.L4.rotations.in(Rotations));
  LoggedTunableNumber elevatorNetPosition =
      new LoggedTunableNumber("/Elevator/NetPosition", ElevatorState.NET.rotations.in(Rotations));

  /* Constructor */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  /* Periodically running code */
  @Override
  public void periodic() {
    if (!isOpenLoop) io.setTargetPosition(currentElevatorState.rotations);
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    double elevatorStage1HeightMeters =
        // rev_of_motor / gear ratio * circumference_of_drum/rev_of_drum  = height in meters
        Units.radiansToRotations(inputs.mainPositionRad)
            / ELEVATOR_GEAR_RATIO
            * (Math.PI * ELEVATOR_SPOOL_DIAMETER.in(Meters));

    double elevatorStage2HeightMeters = elevatorStage1HeightMeters * 2.0;

    Robot.componentPoses[0] = new Pose3d(0.0, 0.0, elevatorStage1HeightMeters, Rotation3d.kZero);
    Robot.componentPoses[1] = new Pose3d(0.0, 0.0, elevatorStage2HeightMeters, Rotation3d.kZero);

    if (!inputs.mainConnected) {
      mainMotorDisconnectedAlert.set(true);
    }

    if (!inputs.followerConnected) {
      followerMotorDisconnectedAlert.set(true);
    }

    if (inputs.forwardLimitSwitchTriggered) {
      forwardLimitSwitchTriggeredAlert.set(true);
    }

    if (inputs.reverseLimitSwitchTriggered) {
      reverseLimitSwitchTriggeredAlert.set(true);
    }

    // driver variables to visualize the elevator state
    SmartDashboard.putBoolean("ZERO", currentElevatorState == ElevatorState.ZERO);
    SmartDashboard.putBoolean("L1", currentElevatorState == ElevatorState.L1);
    SmartDashboard.putBoolean("L2", currentElevatorState == ElevatorState.L2);
    SmartDashboard.putBoolean("STATION", currentElevatorState == ElevatorState.STATION);
    SmartDashboard.putBoolean("L3", currentElevatorState == ElevatorState.L3);
    SmartDashboard.putBoolean("L4", currentElevatorState == ElevatorState.L4);
    SmartDashboard.putBoolean("NET", currentElevatorState == ElevatorState.NET);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        nums -> {
          ElevatorState.PROCESSOR.rotations = Rotations.of(nums[0]);
          ElevatorState.L1.rotations = Rotations.of(nums[1]);
          ElevatorState.L2.rotations = Rotations.of(nums[2]);
          ElevatorState.STATION.rotations = Rotations.of(nums[3]);
          ElevatorState.L3.rotations = Rotations.of(nums[4]);
          ElevatorState.L4.rotations = Rotations.of(nums[5]);
          ElevatorState.NET.rotations = Rotations.of(nums[6]);
        },
        elevatorProcessorPosition,
        elevatorL1Position,
        elevatorL2Position,
        elevatorStationPosition,
        elevatorL3Position,
        elevatorL4Position,
        elevatorNetPosition);
  }

  public Command goToState(ElevatorState targetState) {
    return this.runOnce(
        () -> {
          isOpenLoop = false;
          currentElevatorState = targetState;
        });
  }

  public Command lowerElevatorOpenLoop() {
    return this.runEnd(
        () -> {
          isOpenLoop = true;
          io.setVoltageOpenLoop(Volts.of(-0.1 * 12.0));
        },
        () -> io.holdPosition());
  }

  public Command raiseElevatorOpenLoop() {
    return this.runEnd(
        () -> {
          isOpenLoop = true;
          io.setVoltageOpenLoop(Volts.of(0.1 * 12.0));
        },
        () -> io.holdPosition());
  }

  public Command homeElevator() {
    return new FunctionalCommand(
        () -> {},
        () -> {
          isOpenLoop = true;
          io.setVoltageOpenLoop(Volts.of(-0.1 * 12.0));
        },
        interrupted -> {
          isOpenLoop = false;
          if (interrupted) { // e.g. robot disabled or operator lets go of button
            io.holdPosition();
          }
          if (!interrupted) { // ends normally when bottom limit switch is triggered
            io.zeroPosition();
            currentElevatorState = ElevatorState.ZERO;
          }
        },
        () -> inputs.reverseLimitSwitchTriggered);
  }

  @AutoLogOutput
  public ElevatorState getCurrentElevatorState() {
    return currentElevatorState;
  }

  public Angle getCurrentElevatorPosition() {
    return Radians.of(inputs.mainPositionRad);
  }
}
