package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorState;
import frc.robot.util.LoggedTunableNumber;
import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  /* IO and hardware inputs */
  public final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  /* Connection Alerts */
  private final Alert mainMotorDisconnectedAlert =
      new Alert("Main Elevator Motor Disconnected.", AlertType.kError);
  private final Alert followerMotorDisconnectedAlert =
      new Alert("Follower Elevator Motor Disconnected.", AlertType.kError);

  /* State tracker for current height of the elevator */
  public ElevatorState currentElevatorState = ElevatorState.STATION;

  @AutoLogOutput
  public Trigger isAtTargetState =
      new Trigger(
          () ->
              getCurrentElevatorPosition()
                  .isNear(currentElevatorState.rotations, Rotations.of(0.5)));

  public Trigger isAtL4 =
      new Trigger(
          () -> getCurrentElevatorPosition().isNear(ElevatorState.L4.rotations, Rotations.of(2)));

  @AutoLogOutput
  public Trigger isAboveL1Intermediate =
      new Trigger(() -> getCurrentElevatorPosition().gte(ElevatorState.L1_INTERMEDIATE.rotations));

  public Trigger isAboveL2 =
      new Trigger(() -> getCurrentElevatorPosition().gte(ElevatorState.L2.rotations));

  public Trigger isAboveL3 =
      new Trigger(() -> getCurrentElevatorPosition().gte(ElevatorState.L3.rotations));

  public Trigger isAtStation =
      new Trigger(
          () ->
              getCurrentElevatorPosition()
                  .isNear(ElevatorState.STATION.rotations, Rotations.of(0.25)));

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

  LoggedTunableNumber kP = new LoggedTunableNumber("/Elevator/kP", ELEVATOR_kP);
  LoggedTunableNumber kS = new LoggedTunableNumber("/Elevator/kS", ELEVATOR_kS);
  LoggedTunableNumber kG = new LoggedTunableNumber("/Elevator/kG", ELEVATOR_kG);
  LoggedTunableNumber kV = new LoggedTunableNumber("/Elevator/kV", ELEVATOR_kV);

  LoggedTunableNumber maxVel =
      new LoggedTunableNumber("/Elevator/MaxVel", ELEVATOR_MOTION_MAGIC_CRUISE_VELOCITY);
  LoggedTunableNumber maxAccel =
      new LoggedTunableNumber("/Elevator/MaxAccel", ELEVATOR_MOTION_MAGIC_ACCELERATION);

  /* Constructor */
  public Elevator(ElevatorIO io) {
    this.io = io;

    // failsafe command buttons for manually moving elevator to setpoints regardless of wrist
    // position
    // SmartDashboard.putData("Elevator/Processor", setTargetState(ElevatorState.PROCESSOR));
    // SmartDashboard.putData("Elevator/L1", setTargetState(ElevatorState.L1));
    // SmartDashboard.putData("Elevator/L2", setTargetState(ElevatorState.L2));
    // SmartDashboard.putData("Elevator/Station", setTargetState(ElevatorState.STATION));
    // SmartDashboard.putData("Elevator/L3", setTargetState(ElevatorState.L3));
    // SmartDashboard.putData("Elevator/L4", setTargetState(ElevatorState.L4));
    // SmartDashboard.putData("Elevator/Net", setTargetState(ElevatorState.NET));

    SmartDashboard.putData(this);
  }

  /* Periodically running code */
  @Override
  public void periodic() {
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

    // driver variables to visualize the elevator state
    SmartDashboard.putBoolean("STATION", currentElevatorState == ElevatorState.STATION);
    SmartDashboard.putBoolean("L1", currentElevatorState == ElevatorState.L1);
    SmartDashboard.putBoolean("L2", currentElevatorState == ElevatorState.L2);
    SmartDashboard.putBoolean("L3", currentElevatorState == ElevatorState.L3);
    SmartDashboard.putBoolean("L4", currentElevatorState == ElevatorState.L4);
    SmartDashboard.putBoolean("NET", currentElevatorState == ElevatorState.NET);

    // LoggedTunableNumber.ifChanged(
    //     hashCode(),
    //     nums -> {
    //       ElevatorState.PROCESSOR.rotations = Rotations.of(nums[0]);
    //       ElevatorState.L1.rotations = Rotations.of(nums[1]);
    //       ElevatorState.L2.rotations = Rotations.of(nums[2]);
    //       ElevatorState.STATION.rotations = Rotations.of(nums[3]);
    //       ElevatorState.L3.rotations = Rotations.of(nums[4]);
    //       ElevatorState.L4.rotations = Rotations.of(nums[5]);
    //       ElevatorState.NET.rotations = Rotations.of(nums[6]);

    //       Slot0Configs gains =
    //           new
    // Slot0Configs().withKP(nums[7]).withKV(nums[8]).withKS(nums[9]).withKG(nums[10]);
    //       MotionMagicConfigs motmag =
    //           new MotionMagicConfigs()
    //               .withMotionMagicCruiseVelocity(nums[11])
    //               .withMotionMagicAcceleration(nums[12]);

    //       ((ElevatorIOTalonFX) io).mainElevatorMotor.getConfigurator().apply(gains);
    //       ((ElevatorIOTalonFX) io).followerElevatorMotor.getConfigurator().apply(gains);

    //       ((ElevatorIOTalonFX) io).mainElevatorMotor.getConfigurator().apply(motmag);
    //       ((ElevatorIOTalonFX) io).followerElevatorMotor.getConfigurator().apply(motmag);
    //     },
    //     elevatorProcessorPosition,
    //     elevatorL1Position,
    //     elevatorL2Position,
    //     elevatorStationPosition,
    //     elevatorL3Position,
    //     elevatorL4Position,
    //     elevatorNetPosition,
    //     kP,
    //     kV,
    //     kS,
    //     kG,
    //     maxVel,
    //     maxAccel);
  }

  public Command runTargetState(ElevatorState targetState) {
    return run(
        () -> {
          io.setTargetPosition(targetState.rotations);
          currentElevatorState = targetState;
        });
  }

  public Command toggleElevatorSpeed() {
    return Commands.defer(() -> runOnce(() -> io.toggleMotorProfile()), Set.of(this));
  }

  public void setToAlgaeSpeed() {
    io.setToAlgaeSpeed();
  }

  public void setToCoralSpeed() {
    io.setToCoralSpeed();
  }

  @AutoLogOutput
  public ElevatorState getCurrentElevatorState() {
    return currentElevatorState;
  }

  public Angle getCurrentElevatorPosition() {
    return Radians.of(inputs.mainPositionRad);
  }
}
